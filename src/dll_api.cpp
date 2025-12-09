// src/dll_api.cpp
// TMC5160 FTDI(SPI) DLL API
//
// 要求対応:
// 1) 何かしらのエラー(戻り値!=OK/timeout/invalid arg/IO)が出たら「即時停止」する
//    - 可能なら XTARGET=XACTUAL にして進行中の移動イベントをキャンセル
//    - VMAX=0 + RAMPMODE=HOLD
//    - fault をラッチして以後の動作をブロック(クリアAPIで解除)
// 2) ホーミング処理のロジック(手順)は指定動作になるように修正
// 3) VACTUALは符号付き24bitなので必ず符号拡張（方向判定バグ対策）
// 4) HWリミットヒット時は通常動作では「移動イベントをキャンセル」して確実に止める
//    ★誤ラッチ対策：STOP入力は「動作中の立ち上がり(rising edge)」を起点に確定する
//
// ★今回の修正（RIGHT側ホーミング不具合修正）:
// - RIGHT側で「押下→反転→離れる→反転→再押下で停止完了」が成立しない件を根絶
//   対策：ホーミングをフェーズ分割し、各フェーズで必ず
//     (a) force_hold_stop() で状態を確定
//     (b) SW_MODE をフェーズ用に再設定
//     (c) RAMPSTATイベントをクリア
//     (d) 方向を明示して rotate 再開
//   として、STOP状態が残って回転再開できないパターンを潰す。
// - backoff は target STOP を無効化して離脱し、離脱(OFF)を検出したら
//   即 SW_MODE(hard-stop) + VSTOP=0 + event clear の上で 2nd approach を開始する。

#include "tmc5160_dll.h"
#include "tmc5160.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <mutex>

#ifdef _WIN32
  #include <windows.h>
#endif

#ifdef _WIN32
  #include <mmsystem.h>
  #include <atomic>
  #pragma comment(lib, "winmm.lib")
#endif

#ifdef _WIN32
static std::atomic<int> g_timeperiod_ref{0};

static inline void winmm_acquire_1ms() {
  if (g_timeperiod_ref.fetch_add(1) == 0) timeBeginPeriod(1);
}
static inline void winmm_release_1ms() {
  if (g_timeperiod_ref.fetch_sub(1) == 1) timeEndPeriod(1);
}
#endif

#include <ftd2xx.h>

// ============================================================================
// Small yields
// ============================================================================
static inline void tiny_yield() {
#ifdef _WIN32
  Sleep(1);
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
}

static inline void poll_yield(uint32_t& ctr) {
#ifdef _WIN32
  if ((ctr++ & 0x0F) == 0) {
    Sleep(1);
  } else {
    if (!SwitchToThread()) Sleep(0);
  }
#else
  if ((ctr++ & 0x0F) == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  else std::this_thread::yield();
#endif
}

static inline bool ramp_pos_reached(uint32_t rs) { return ((rs >> 9) & 1u) != 0; }

// ============================================================================
// Error strings
// ============================================================================
static thread_local std::string g_last_err;
static void setGlobalErr(const std::string& s) { g_last_err = s; }

static void copyErr(const std::string& s, char* out, uint32_t out_len) {
  if (!out || out_len == 0) return;
  size_t n = s.size();
  if (n >= out_len) n = out_len - 1;
  std::memcpy(out, s.data(), n);
  out[n] = 0;
}

// ============================================================================
// Context
// ============================================================================
struct Context {
  Tmc5160 tmc;
  std::string last_err;

  bool fault_latched = false;
  bool homing_active = false;
  std::string fault_reason;

  // Normal motion
  bool limits_active_low = true;
  bool limits_softstop   = false;

  // STOP status history (for rising-edge based detection)
  uint32_t last_stop_status = 0;

  // Debounce/confirm counters (only after rising-edge)
  uint32_t last_event_bits = 0;
  uint8_t l_status_cnt = 0;
  uint8_t r_status_cnt = 0;

  // cached ramp params (for restoring after HOLD/VMAX=0 stops)
  bool     have_cached_vmax = false;
  uint32_t cached_vmax      = 0;

  // ==== SPI bus lock (prevents interleaved writes across threads) ====
  std::recursive_mutex io_mtx;

  // ==== Panic kill state (driver output off) ====
  bool driver_killed = false;

  bool saved_chopconf_valid = false;
  uint32_t saved_chopconf = 0;

  bool saved_ihold_valid = false;
  uint32_t saved_ihold_irun = 0;
};

static void setErr(Context* c, const std::string& s) { if (c) c->last_err = s; setGlobalErr(s); }

// handle -> Context*
static inline Context* ctx_from_handle(TMC5160_Handle h) { return h ? (Context*)h : nullptr; }

// ============================================================================
// Registers
// ============================================================================
static constexpr uint8_t REG_GCONF      = 0x00;
static constexpr uint8_t REG_GSTAT      = 0x01;
static constexpr uint8_t REG_IOIN       = 0x04;

static constexpr uint8_t REG_IHOLD_IRUN = 0x10;
static constexpr uint8_t REG_TPOWERDOWN = 0x11;

static constexpr uint8_t REG_RAMPMODE   = 0x20;
static constexpr uint8_t REG_XACTUAL    = 0x21;
static constexpr uint8_t REG_VACTUAL    = 0x22;

static constexpr uint8_t REG_VSTART     = 0x23;
static constexpr uint8_t REG_A1         = 0x24;
static constexpr uint8_t REG_V1         = 0x25;
static constexpr uint8_t REG_AMAX       = 0x26;
static constexpr uint8_t REG_VMAX       = 0x27;
static constexpr uint8_t REG_DMAX       = 0x28;
static constexpr uint8_t REG_D1         = 0x2A;
static constexpr uint8_t REG_VSTOP      = 0x2B;
static constexpr uint8_t REG_XTARGET    = 0x2D;

static constexpr uint8_t REG_SW_MODE    = 0x34;
static constexpr uint8_t REG_RAMPSTAT   = 0x35;
static constexpr uint8_t REG_XLATCH     = 0x36;

static constexpr uint8_t REG_ENCMODE    = 0x38;
static constexpr uint8_t REG_X_ENC      = 0x39;

static constexpr uint8_t REG_CHOPCONF   = 0x6C;

// RAMPMODE
static constexpr uint32_t RM_POSITION = 0;
static constexpr uint32_t RM_VEL_POS  = 1;
static constexpr uint32_t RM_VEL_NEG  = 2;
static constexpr uint32_t RM_HOLD     = 3;

// SW_MODE bits
static constexpr uint32_t SW_STOP_L_EN      = (1u << 0);
static constexpr uint32_t SW_STOP_R_EN      = (1u << 1);
static constexpr uint32_t SW_POL_STOP_L     = (1u << 2);
static constexpr uint32_t SW_POL_STOP_R     = (1u << 3);
static constexpr uint32_t SW_LATCH_L_ACTIVE = (1u << 5);
static constexpr uint32_t SW_LATCH_R_ACTIVE = (1u << 7);
static constexpr uint32_t SW_EN_SOFTSTOP    = (1u << 11);

// RAMPSTAT bits
static constexpr uint32_t RS_STATUS_STOP_L  = (1u << 0);
static constexpr uint32_t RS_STATUS_STOP_R  = (1u << 1);
static constexpr uint32_t RS_LATCH_L        = (1u << 2);
static constexpr uint32_t RS_LATCH_R        = (1u << 3);
static constexpr uint32_t RS_EVENT_STOP_L   = (1u << 4);
static constexpr uint32_t RS_EVENT_STOP_R   = (1u << 5);
static constexpr uint32_t RS_VZERO          = (1u << 10);

// ============================================================================
// BEST-EFFORT stop primitives  (STRICT: retry + verify + wait)
// ============================================================================

static inline bool best_effort_read(Context* c, uint8_t addr, uint32_t* out) {
  if (!c || !out) return false;
  uint8_t st = 0;
  std::string e;
  uint32_t v = 0;
  int r = c->tmc.readReg(addr, &v, &st, e);
  if (r) return false;
  *out = v;
  return true;
}

static inline void best_effort_write(Context* c, uint8_t addr, uint32_t v) {
  if (!c) return;
  uint8_t st = 0;
  std::string e;
  (void)c->tmc.writeReg(addr, v, &st, e);
}

static inline void driver_kill_outputs_best_effort(Context* c) {
  if (!c) return;

  // save CHOPCONF / IHOLD_IRUN once
  if (!c->saved_chopconf_valid) {
    uint32_t v = 0;
    if (best_effort_read(c, REG_CHOPCONF, &v)) {
      c->saved_chopconf = v;
      c->saved_chopconf_valid = true;
    }
  }
  if (!c->saved_ihold_valid) {
    uint32_t v = 0;
    if (best_effort_read(c, REG_IHOLD_IRUN, &v)) {
      c->saved_ihold_irun = v;
      c->saved_ihold_valid = true;
    }
  }

  // 1) disable driver output: CHOPCONF.TOFF = 0 (bits[3:0]=0)
  uint32_t chop = 0;
  if (best_effort_read(c, REG_CHOPCONF, &chop)) {
    chop &= ~0xFu;
    best_effort_write(c, REG_CHOPCONF, chop);
  }

  // 2) set IHOLD/IRUN = 0 (extra safety)
  uint32_t ih = 0;
  if (best_effort_read(c, REG_IHOLD_IRUN, &ih)) {
    ih &= ~((0x1Fu << 0) | (0x1Fu << 8)); // IHOLD=0, IRUN=0
    best_effort_write(c, REG_IHOLD_IRUN, ih);
  }

  c->driver_killed = true;
}

static inline void driver_restore_outputs_best_effort(Context* c) {
  if (!c) return;
  if (!c->driver_killed) return;

  if (c->saved_chopconf_valid) best_effort_write(c, REG_CHOPCONF, c->saved_chopconf);
  if (c->saved_ihold_valid)    best_effort_write(c, REG_IHOLD_IRUN, c->saved_ihold_irun);

  c->driver_killed = false;
}

static inline void latch_fault(Context* c, const char* reason) {
  if (!c) return;
  if (!c->fault_latched) {
    c->fault_latched = true;
    c->fault_reason = reason ? reason : "fault";
  }
}

static inline int32_t sign_extend24_u32(uint32_t v) {
  v &= 0x00FFFFFFu;
  if (v & 0x00800000u) v |= 0xFF000000u;
  return (int32_t)v;
}

// ★ここが肝：
// - XTARGET=XACTUAL で「進行中イベント」をキャンセル
// - RAMPMODE=HOLD + VMAX=0 (+ VSTOP=0) を確実に入れる（読戻し確認 + リトライ）
// - 最後に VZERO になるまで待つ（動作が残る/書込み失敗を潰す）
static inline bool cancel_and_hold_strict(Context* c, uint32_t wait_ms = 2000, bool allow_kill = true) {
  if (!c) return false;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  uint32_t xact = 0;
  bool have_xact = best_effort_read(c, REG_XACTUAL, &xact);

  auto reassert_stop = [&]() {
    if (have_xact) best_effort_write(c, REG_XTARGET, xact); // cancel queued move
    best_effort_write(c, REG_RAMPMODE, RM_HOLD);
    best_effort_write(c, REG_VMAX, 0);
    best_effort_write(c, REG_VSTOP, 0);
    best_effort_write(c, REG_RAMPSTAT, (RS_LATCH_L|RS_LATCH_R|RS_EVENT_STOP_L|RS_EVENT_STOP_R));
  };

  // strong retry until confirmed (more than before)
  bool confirmed = false;
  for (int attempt = 0; attempt < 20; ++attempt) {
    reassert_stop();
    tiny_yield();

    uint32_t rm = 0, vmax = 0;
    bool ok_rm = best_effort_read(c, REG_RAMPMODE, &rm);
    bool ok_v  = best_effort_read(c, REG_VMAX, &vmax);
    if (ok_rm && ok_v && ((rm & 0x3u) == RM_HOLD) && (vmax == 0)) {
      confirmed = true;
      break;
    }
  }

  auto t0 = std::chrono::steady_clock::now();
  uint32_t last_reassert_ms = 0;

  while (true) {
    uint32_t rs_u = 0, v_u = 0;
    bool ok_rs = best_effort_read(c, REG_RAMPSTAT, &rs_u);
    bool ok_v  = best_effort_read(c, REG_VACTUAL, &v_u);
    const int32_t vact = ok_v ? sign_extend24_u32(v_u) : 0;

    if ((ok_rs && (rs_u & RS_VZERO)) || (ok_v && (std::abs(vact) <= 1))) {
      return true;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();

    // keep hammering stop state (prevents “overwritten by other writes” and recovers from sporadic SPI fails)
    if (ms - last_reassert_ms >= 20) {
      reassert_stop();
      last_reassert_ms = ms;
    }

    if (ms >= wait_ms) break;
    tiny_yield();
  }

  // Not stopped within wait_ms -> PANIC KILL (physical stop)
  if (allow_kill) {
    driver_kill_outputs_best_effort(c);
    reassert_stop();
    tiny_yield();
  }

  return false;
}

// ============================================================================
// FAST stop primitive (for TIMEOUT paths): command HOLD immediately and return quickly
// - short verify window, then (optionally) panic-kill outputs
// ============================================================================
static inline bool cancel_and_hold_fast(Context* c, uint32_t wait_ms = 80, bool allow_kill = true) {
  if (!c) return false;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  uint32_t xact = 0;
  bool have_xact = best_effort_read(c, REG_XACTUAL, &xact);

  auto issue_stop = [&]() {
    if (have_xact) best_effort_write(c, REG_XTARGET, xact); // cancel queued move
    best_effort_write(c, REG_RAMPMODE, RM_HOLD);
    best_effort_write(c, REG_VMAX, 0);
    best_effort_write(c, REG_VSTOP, 0);
    // clear edge/event bits so they don't "stick" into next phase
    best_effort_write(c, REG_RAMPSTAT, (RS_LATCH_L | RS_LATCH_R | RS_EVENT_STOP_L | RS_EVENT_STOP_R));
  };

  // まず即時に停止コマンドを叩く
  issue_stop();

  // 可能なら軽くレジスタ確認（長く待たない）
  for (int i = 0; i < 3; ++i) {
    uint32_t rm = 0, vmax = 0;
    bool ok_rm = best_effort_read(c, REG_RAMPMODE, &rm);
    bool ok_v  = best_effort_read(c, REG_VMAX, &vmax);
    if (ok_rm && ok_v && ((rm & 0x3u) == RM_HOLD) && (vmax == 0)) break;
    issue_stop();
    tiny_yield();
  }

  auto t0 = std::chrono::steady_clock::now();
  uint32_t last_reassert_ms = 0;

  while (true) {
    uint32_t rs_u = 0, v_u = 0;
    bool ok_rs = best_effort_read(c, REG_RAMPSTAT, &rs_u);
    bool ok_v  = best_effort_read(c, REG_VACTUAL, &v_u);
    const int32_t vact = ok_v ? sign_extend24_u32(v_u) : 0;

    if ((ok_rs && (rs_u & RS_VZERO)) || (ok_v && (std::abs(vact) <= 1))) {
      return true; // すぐ止まった
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= wait_ms) break;

    // 念のため短周期で再assert（SPI抜け/他書込み競合対策）
    if (ms - last_reassert_ms >= 10) {
      issue_stop();
      last_reassert_ms = ms;
    }

    tiny_yield();
  }

  // ここまでで止まり切らない => timeout経路では「返りを遅らせない」ため物理KILL優先
  if (allow_kill) {
    driver_kill_outputs_best_effort(c);
    issue_stop();
    tiny_yield();
  }

  return false;
}

// 旧best_effort_cancel_and_hold() を「timeoutでも戻りが遅くならない版」に差替
static inline void best_effort_cancel_and_hold(Context* c) {
  // TIMEOUT系の経路で多用されるので、長い standstill 待ちをしない
  (void)cancel_and_hold_fast(c, /*wait_ms=*/80, /*allow_kill=*/true);
}

static int32_t stop_on_error(Context* c, int32_t rc, const char* api, const std::string& msg) {
  if (c) {
    setErr(c, msg);
    latch_fault(c, api ? api : "error");

    // ★ここが本命：timeout時は「停止確認で2秒待つ」せいでエラー返却が遅い
    // -> timeoutだけ高速停止にして、即returnする
    if (rc == TMC5160_ERR_TIMEOUT) {
      (void)cancel_and_hold_fast(c, /*wait_ms=*/80, /*allow_kill=*/true);
    } else {
      // IO系などは従来通り「確実に止める」
      (void)cancel_and_hold_strict(c, /*wait_ms=*/2000, /*allow_kill=*/true);
    }
  } else {
    setGlobalErr(msg);
  }
  return rc;
}

static int32_t rejectIfFaulted(Context* c, const char* api) {
  if (!c) return TMC5160_ERR_INVALID_ARG;
  if (!c->fault_latched) return 0;

  // すでに「limit hit ...」等で last_err がセット済みなら、それを保持して返す
  if (c->last_err.empty()) {
    std::string msg = std::string(api ? api : "api") +
                      ": blocked (fault latched: " + c->fault_reason + ")";
    setErr(c, msg);
  } else {
    // global側も last_err に揃える（呼び出し元で last_error_global を使う場合のため）
    setGlobalErr(c->last_err);
  }

  best_effort_cancel_and_hold(c);
  return TMC5160_ERR_IO;
}

// ============================================================================
// RAW register access (NO safety inside)
// ============================================================================
static int32_t raw_write_reg(Context* c, uint8_t addr, uint32_t value, uint8_t* out_status) {
  if (!c) return TMC5160_ERR_INVALID_ARG;
  std::string err;
  int r = c->tmc.writeReg(addr, value, out_status, err);
  if (r) return stop_on_error(c, TMC5160_ERR_IO, "raw_write_reg", err.empty() ? "writeReg failed" : err);
  return TMC5160_OK;
}

static int32_t raw_read_reg(Context* c, uint8_t addr, uint32_t* out_value, uint8_t* out_status) {
  if (!c || !out_value) return TMC5160_ERR_INVALID_ARG;
  std::string err;
  int r = c->tmc.readReg(addr, out_value, out_status, err);
  if (r) return stop_on_error(c, TMC5160_ERR_IO, "raw_read_reg", err.empty() ? "readReg failed" : err);
  return TMC5160_OK;
}

static int32_t rd_u32(TMC5160_Handle h, uint8_t addr, uint32_t* out) {
  auto* c = ctx_from_handle(h);
  if (!c || !out) return TMC5160_ERR_INVALID_ARG;
  uint8_t st = 0; uint32_t v = 0;
  int32_t rc = raw_read_reg(c, addr, &v, &st);
  if (rc != 0) return rc;
  *out = v;
  return TMC5160_OK;
}

static int32_t wr_u32(TMC5160_Handle h, uint8_t addr, uint32_t v) {
  auto* c = ctx_from_handle(h);
  if (!c) return TMC5160_ERR_INVALID_ARG;
  uint8_t st = 0;
  return raw_write_reg(c, addr, v, &st);
}

static inline int32_t sign_extend24(uint32_t v) {
  v &= 0x00FFFFFFu;
  if (v & 0x00800000u) v |= 0xFF000000u;
  return (int32_t)v;
}

static int32_t rd_s24(TMC5160_Handle h, uint8_t addr, int32_t* out) {
  auto* c = ctx_from_handle(h);
  if (!c || !out) return TMC5160_ERR_INVALID_ARG;
  uint8_t st = 0; uint32_t v = 0;
  int32_t rc = raw_read_reg(c, addr, &v, &st);
  if (rc != 0) return rc;
  *out = sign_extend24(v);
  return TMC5160_OK;
}

static int32_t rd_s32(TMC5160_Handle h, uint8_t addr, int32_t* out) {
  uint32_t v=0;
  int32_t rc = rd_u32(h, addr, &v);
  if (rc != 0) return rc;
  *out = (int32_t)v;
  return TMC5160_OK;
}

static inline bool is_right_dir(int32_t dir) { return dir > 0; }

// IOIN bit0=REFL bit1=REFR
static inline bool ioin_level(uint32_t ioin, bool right) {
  return (ioin & (right ? (1u << 1) : (1u << 0))) != 0;
}

// homing 判定は SW_MODE の POL に依存せず、flags の active_low を直接使う
static inline bool home_pressed(uint32_t ioin, bool right, bool active_low) {
  const bool level = ioin_level(ioin, right);   // IOINの生レベル(1/0)
  return active_low ? !level : level;           // active_lowなら 0=押下
}

// SW_MODE polarity-based interpretation
// (この実装は「POL bit=1 => LOW active」として扱う：あなたの既存挙動(LEFTが正常)に合わせる)
static inline bool pressed_from_ioin_sw(uint32_t ioin, bool right, uint32_t sw_mode) {
  const bool level = ioin_level(ioin, right);
  const bool pol_low_active = (sw_mode & (right ? SW_POL_STOP_R : SW_POL_STOP_L)) != 0;
  return pol_low_active ? !level : level;
}

static int32_t clear_rampstat_events(TMC5160_Handle h) {
  uint32_t m = RS_LATCH_L | RS_LATCH_R | RS_EVENT_STOP_L | RS_EVENT_STOP_R;
  return wr_u32(h, REG_RAMPSTAT, m);
}

// ============================================================================
// Stop / wait
// ============================================================================
static int32_t wait_stopped(TMC5160_Handle h, uint32_t timeout_ms) {
  auto* c = ctx_from_handle(h);
  auto t0 = std::chrono::steady_clock::now();

  while (true) {
    uint32_t rs = 0;
    int32_t vact = 0;
    int32_t rc = rd_u32(h, REG_RAMPSTAT, &rs);
    if (rc != 0) return rc;
    rc = rd_s24(h, REG_VACTUAL, &vact);
    if (rc != 0) return rc;

    if ((rs & RS_VZERO) || (std::abs(vact) <= 1)) return TMC5160_OK;

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) {
      // ★timeout = 即停止（戻り値はtimeoutのまま）
      if (c) best_effort_cancel_and_hold(c);
      return TMC5160_ERR_TIMEOUT;
    }

    tiny_yield();
  }
}

static int32_t force_hold_stop(TMC5160_Handle h, uint32_t timeout_ms) {
  int32_t rc = wr_u32(h, REG_VMAX, 0);
  if (rc != 0) return rc;
  rc = wr_u32(h, REG_RAMPMODE, RM_HOLD);
  if (rc != 0) return rc;
  return wait_stopped(h, timeout_ms);
}

// ============================================================================
// Motion epoch / limit safety (NORMAL motion)
// ============================================================================
static inline void begin_motion_epoch(TMC5160_Handle h, Context* c) {
  (void)clear_rampstat_events(h);
  if (c) {
    c->last_event_bits  = 0;
    c->last_stop_status = 0;
    c->l_status_cnt = c->r_status_cnt = 0;
  }
}

static uint32_t detect_limit_hit_mask(TMC5160_Handle h, Context* c) {
  uint32_t sw = 0;
  if (rd_u32(h, REG_SW_MODE, &sw) != 0) return 0;

  const bool l_en = (sw & SW_STOP_L_EN) != 0;
  const bool r_en = (sw & SW_STOP_R_EN) != 0;
  if (!l_en && !r_en) return 0;

  uint32_t rs = 0;
  if (rd_u32(h, REG_RAMPSTAT, &rs) != 0) return 0;

  // IOINも読めるなら「実ピン押下」を併用（読めない場合はRAMPSTATのstatusのみ）
  uint32_t ioin = 0;
  const bool have_ioin = (rd_u32(h, REG_IOIN, &ioin) == 0);

  auto pressedL = [&]() -> bool {
    if (have_ioin) return pressed_from_ioin_sw(ioin, /*right=*/false, sw);
    return (rs & RS_STATUS_STOP_L) != 0;
  };
  auto pressedR = [&]() -> bool {
    if (have_ioin) return pressed_from_ioin_sw(ioin, /*right=*/true, sw);
    return (rs & RS_STATUS_STOP_R) != 0;
  };

  // 方向推定：VACTUAL優先。ダメなら RAMPMODE。POSITIONなら XTARGET vs XACTUAL。
  int32_t vact = 0;
  (void)rd_s24(h, REG_VACTUAL, &vact);

  enum { DIR_NONE = 0, DIR_LEFT = -1, DIR_RIGHT = +1 };
  int dir = DIR_NONE;

  if (vact > +1) dir = DIR_RIGHT;
  else if (vact < -1) dir = DIR_LEFT;
  else {
    uint32_t rm = 0;
    if (rd_u32(h, REG_RAMPMODE, &rm) == 0) {
      if (rm == RM_VEL_POS) dir = DIR_RIGHT;
      else if (rm == RM_VEL_NEG) dir = DIR_LEFT;
      else if (rm == RM_POSITION) {
        int32_t xa = 0, xt = 0;
        if (rd_s32(h, REG_XACTUAL, &xa) == 0 && rd_s32(h, REG_XTARGET, &xt) == 0) {
          if (xt > xa) dir = DIR_RIGHT;
          else if (xt < xa) dir = DIR_LEFT;
        }
      }
    }
  }

  // EVENT rising（履歴）
  const uint32_t ev_cur  = (rs & (RS_EVENT_STOP_L | RS_EVENT_STOP_R));
  const uint32_t ev_prev = c ? c->last_event_bits : 0;
  const uint32_t ev_rise = ev_cur & ~ev_prev;
  if (c) c->last_event_bits = ev_cur;

  // STATUS rising（デバウンス用）
  const uint32_t cur_status  = (rs & (RS_STATUS_STOP_L | RS_STATUS_STOP_R));
  const uint32_t prev_status = c ? c->last_stop_status : 0;
  const uint32_t rising_st   = cur_status & ~prev_status;
  if (c) c->last_stop_status = cur_status;

  // 方向が違う側のカウンタは落とす（誤ラッチ対策）
  if (c) {
    if (dir != DIR_LEFT)  c->l_status_cnt = 0;
    if (dir != DIR_RIGHT) c->r_status_cnt = 0;
  }

  uint32_t hit = 0;
  const int STABLE_REQ = 2; // ~2ms安定で確定（必要なら増やす）

  // LEFT hit（左へ向かう意図がある時だけ）
  if (l_en && dir == DIR_LEFT) {
    const bool st = (rs & RS_STATUS_STOP_L) != 0;

    // event rising は最優先で確定
    if ((ev_rise & RS_EVENT_STOP_L) && st) hit |= 1u;

    // pressed+status の安定判定
    if (pressedL() && st) {
      if (!c) {
        hit |= 1u;
      } else {
        if (rising_st & RS_STATUS_STOP_L) c->l_status_cnt = 1;
        else if (c->l_status_cnt > 0 && c->l_status_cnt < 250) c->l_status_cnt++;
        if (c->l_status_cnt >= STABLE_REQ) hit |= 1u;
      }
    } else {
      if (c) c->l_status_cnt = 0;
    }
  }

  // RIGHT hit（右へ向かう意図がある時だけ）
  if (r_en && dir == DIR_RIGHT) {
    const bool st = (rs & RS_STATUS_STOP_R) != 0;

    if ((ev_rise & RS_EVENT_STOP_R) && st) hit |= 2u;

    if (pressedR() && st) {
      if (!c) {
        hit |= 2u;
      } else {
        if (rising_st & RS_STATUS_STOP_R) c->r_status_cnt = 1;
        else if (c->r_status_cnt > 0 && c->r_status_cnt < 250) c->r_status_cnt++;
        if (c->r_status_cnt >= STABLE_REQ) hit |= 2u;
      }
    } else {
      if (c) c->r_status_cnt = 0;
    }
  }

  return hit;
}

static void apply_limit_safety_always(TMC5160_Handle h) {
  if (!h) return;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  if (c->fault_latched) {
    best_effort_cancel_and_hold(c);
    return;
  }

  uint32_t hit = detect_limit_hit_mask(h, c);
  if (!hit) return;

  // 進行中イベントをキャンセルして確実停止
  best_effort_cancel_and_hold(c);

  // home / move_to_limit 中は「止める」だけ（fault latch しない）
  if (c->homing_active) {
    return;
  }

  // 通常動作は fault latch + エラー文字列
  if (hit == 1u) {
    setErr(c, "limit hit (L): motion canceled");
    latch_fault(c, "limit hit (L)");
  } else if (hit == 2u) {
    setErr(c, "limit hit (R): motion canceled");
    latch_fault(c, "limit hit (R)");
  } else {
    setErr(c, "limit hit (L+R): motion canceled");
    latch_fault(c, "limit hit (L+R)");
  }
}

// ============================================================================
// HW limit arming for NORMAL motion (skip during homing)
// ============================================================================
static int32_t arm_hw_limits_for_normal_motion(TMC5160_Handle h, Context* c) {
  if (!h || !c) return TMC5160_ERR_INVALID_ARG;
  if (c->homing_active) return TMC5160_OK;

  uint32_t sw = 0;
  int32_t rc = rd_u32(h, REG_SW_MODE, &sw);
  if (rc != 0) return rc;

  uint32_t sw_new = sw;
  sw_new |= (SW_STOP_L_EN | SW_STOP_R_EN);

  if (c->limits_active_low) sw_new |= (SW_POL_STOP_L | SW_POL_STOP_R);
  else                     sw_new &= ~(SW_POL_STOP_L | SW_POL_STOP_R);

  if (c->limits_softstop) sw_new |= SW_EN_SOFTSTOP;
  else                   sw_new &= ~SW_EN_SOFTSTOP;

  if (sw_new != sw) {
    rc = wr_u32(h, REG_SW_MODE, sw_new);
    if (rc != 0) return rc;
    (void)clear_rampstat_events(h);
  }

  return TMC5160_OK;
}

// ============================================================================
// Ensure VMAX restored for position mode
// ============================================================================
static int32_t ensure_vmax_restored_for_position(TMC5160_Handle h, Context* c) {
  if (!h || !c) return TMC5160_ERR_INVALID_ARG;
  if (!c->have_cached_vmax || c->cached_vmax == 0) {
    return stop_on_error(c, TMC5160_ERR_INVALID_ARG, "ensure_vmax",
                         "ensure_vmax: ramp params not set (cached VMAX is 0)");
  }
  uint32_t cur_vmax = 0;
  int32_t rc = rd_u32(h, REG_VMAX, &cur_vmax);
  if (rc != 0) return rc;
  if (cur_vmax == 0) {
    rc = wr_u32(h, REG_VMAX, c->cached_vmax);
    if (rc != 0) return rc;
  }
  return TMC5160_OK;
}

// ============================================================================
// Homing helper: rotate with vmax
// ============================================================================
static int32_t start_rotate_with_vmax(TMC5160_Handle h, int32_t vel) {
  if (vel == 0) {
    int32_t rc = wr_u32(h, REG_VMAX, 0);
    if (rc != 0) return rc;
    return wr_u32(h, REG_RAMPMODE, RM_HOLD);
  }
  const uint32_t v = (uint32_t)std::max(1, std::abs(vel));
  int32_t rc = wr_u32(h, REG_VMAX, v);
  if (rc != 0) return rc;
  rc = wr_u32(h, REG_RAMPMODE, (vel > 0) ? RM_VEL_POS : RM_VEL_NEG);
  if (rc != 0) return rc;
  return wr_u32(h, REG_VMAX, v);
}

// ============================================================================
// Homing helper: wait for "press" on target (and error if other pressed)
// - requires "armed" by observing target released stable first
// ============================================================================
static int32_t wait_press_armed(TMC5160_Handle h,
                               bool target_right,
                               uint32_t sw_for_polarity,
                               uint32_t timeout_ms)
{
  auto* c = ctx_from_handle(h);
  if (!c) return TMC5160_ERR_INVALID_ARG;

  auto t0 = std::chrono::steady_clock::now();

  const int ARM_RELEASE_REQ  = 10; // 10ms
  const int PRESS_STABLE_REQ = 10; // 10ms
  const int OTHER_STABLE_REQ = 10;

  bool armed = false;
  int  rel_cnt = 0;
  int  prs_cnt = 0;
  int  oth_cnt = 0;

  uint32_t last_ev = 0;

  const uint32_t tgt_st = target_right ? RS_STATUS_STOP_R : RS_STATUS_STOP_L;
  const uint32_t oth_st = target_right ? RS_STATUS_STOP_L : RS_STATUS_STOP_R;
  const uint32_t tgt_ev = target_right ? RS_EVENT_STOP_R  : RS_EVENT_STOP_L;
  const uint32_t oth_ev = target_right ? RS_EVENT_STOP_L  : RS_EVENT_STOP_R;

  while (true) {
    if (c->fault_latched) return rejectIfFaulted(c, "home(wait_press)");

    uint32_t rs = 0;
    int32_t rc = rd_u32(h, REG_RAMPSTAT, &rs);
    if (rc != 0) return rc;

    const uint32_t ev_cur  = rs & (RS_EVENT_STOP_L | RS_EVENT_STOP_R);
    const uint32_t ev_rise = ev_cur & ~last_ev;
    last_ev = ev_cur;

    uint32_t ioin = 0;
    rc = rd_u32(h, REG_IOIN, &ioin);
    if (rc != 0) return rc;

    const bool tgt_pressed = pressed_from_ioin_sw(ioin,  target_right, sw_for_polarity);
    const bool oth_pressed = pressed_from_ioin_sw(ioin, !target_right, sw_for_polarity);

    const bool tgt_status = (rs & tgt_st) != 0;
    const bool oth_status = (rs & oth_st) != 0;

    // other pressed => error (stable or event)
    if (oth_pressed && oth_status) {
      if (++oth_cnt >= OTHER_STABLE_REQ) return TMC5160_ERR_IO;
    } else {
      oth_cnt = 0;
    }
    if (armed && (ev_rise & oth_ev) && oth_status) return TMC5160_ERR_IO;

    // arm by seeing release stable
    if (!armed) {
      if (!tgt_pressed && !tgt_status) {
        if (++rel_cnt >= ARM_RELEASE_REQ) armed = true;
      } else {
        rel_cnt = 0;
      }
    } else {
      // accept by event rising (best) or pressed stable with status
      if ((ev_rise & tgt_ev) && tgt_status) return TMC5160_OK;

      if (tgt_pressed && tgt_status) {
        if (++prs_cnt >= PRESS_STABLE_REQ) return TMC5160_OK;
      } else {
        prs_cnt = 0;
      }
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) {
      // ★timeout = 即停止（戻り値はtimeoutのまま）
      best_effort_cancel_and_hold(c);
      return TMC5160_ERR_TIMEOUT;
    }

    tiny_yield();
  }
}

// ============================================================================
// Exports: error accessors
// ============================================================================
TMC_API int32_t tmc5160_last_error_global(char* out, uint32_t out_len) {
  copyErr(g_last_err, out, out_len);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_last_error(TMC5160_Handle h, char* out, uint32_t out_len) {
  if (!h) { copyErr("", out, out_len); return TMC5160_ERR_INVALID_ARG; }
  copyErr(((Context*)h)->last_err, out, out_len);
  return TMC5160_OK;
}

// ============================================================================
// Enumerate devices (FTDI)
// ============================================================================
TMC_API int32_t tmc5160_get_device_count(uint32_t* out_count) {
  if (!out_count) return TMC5160_ERR_INVALID_ARG;
  DWORD n = 0;
  FT_STATUS st = FT_CreateDeviceInfoList(&n);
  if (st != FT_OK) { setGlobalErr("FT_CreateDeviceInfoList failed"); return TMC5160_ERR_IO; }
  *out_count = (uint32_t)n;
  setGlobalErr("");
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_get_device_info(uint32_t index,
                                        char* out_serial, uint32_t serial_len,
                                        char* out_desc,   uint32_t desc_len) {
  DWORD flags=0, type=0, id=0, loc=0;
  char serial[16] = {0};
  char desc[64] = {0};
  FT_HANDLE tmp = nullptr;

  FT_STATUS st = FT_GetDeviceInfoDetail((DWORD)index, &flags, &type, &id, &loc, serial, desc, &tmp);
  if (st != FT_OK) { setGlobalErr("FT_GetDeviceInfoDetail failed"); return TMC5160_ERR_IO; }

  if (out_serial && serial_len) {
    size_t n = std::strlen(serial);
    if (n >= serial_len) n = serial_len - 1;
    std::memcpy(out_serial, serial, n); out_serial[n] = 0;
  }
  if (out_desc && desc_len) {
    size_t n = std::strlen(desc);
    if (n >= desc_len) n = desc_len - 1;
    std::memcpy(out_desc, desc, n); out_desc[n] = 0;
  }
  setGlobalErr("");
  return TMC5160_OK;
}

// ============================================================================
// Open/Close
// ============================================================================
TMC_API int32_t tmc5160_open_index(uint32_t device_index, uint32_t spi_hz, TMC5160_Handle* out_h) {
  if (!out_h) return TMC5160_ERR_INVALID_ARG;
  *out_h = nullptr;

  auto* c = new Context();
  std::string err;
  int r = c->tmc.openIndex(device_index, spi_hz, err);
  if (r) {
    setErr(c, err.empty() ? "openIndex failed" : err);
    delete c;
    return TMC5160_ERR_IO;
  }

#ifdef _WIN32
  winmm_acquire_1ms();
#endif

  // safe stop on open
  {
    int32_t rc = force_hold_stop((TMC5160_Handle)c, 2000);
    if (rc != 0) {
      stop_on_error(c, rc, "open_index", "force_stop failed on open");
      c->tmc.close();
#ifdef _WIN32
      winmm_release_1ms();
#endif
      delete c;
      return rc;
    }
  }

  (void)arm_hw_limits_for_normal_motion((TMC5160_Handle)c, c);

  c->fault_latched = false;
  c->fault_reason.clear();
  setErr(c, "");
  *out_h = (TMC5160_Handle)c;
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_open_serial(const char* serial, uint32_t spi_hz, TMC5160_Handle* out_h) {
  if (!out_h || !serial) return TMC5160_ERR_INVALID_ARG;
  *out_h = nullptr;

  auto* c = new Context();
  std::string err;
  int r = c->tmc.openSerial(serial, spi_hz, err);
  if (r) {
    setErr(c, err.empty() ? "openSerial failed" : err);
    delete c;
    return TMC5160_ERR_IO;
  }

#ifdef _WIN32
  winmm_acquire_1ms();
#endif

  {
    int32_t rc = force_hold_stop((TMC5160_Handle)c, 2000);
    if (rc != 0) {
      stop_on_error(c, rc, "open_serial", "force_stop failed on open");
      c->tmc.close();
#ifdef _WIN32
      winmm_release_1ms();
#endif
      delete c;
      return rc;
    }
  }

  (void)arm_hw_limits_for_normal_motion((TMC5160_Handle)c, c);

  c->fault_latched = false;
  c->fault_reason.clear();
  setErr(c, "");
  *out_h = (TMC5160_Handle)c;
  return TMC5160_OK;
}

TMC_API void tmc5160_close(TMC5160_Handle h) {
  if (!h) return;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);
  best_effort_cancel_and_hold(c);
  c->tmc.close();
#ifdef _WIN32
  winmm_release_1ms();
#endif
  delete c;
}

// ============================================================================
// Low-level read/write (exported)
// ============================================================================
TMC_API int32_t tmc5160_write_reg(TMC5160_Handle h, uint8_t addr, uint32_t value, uint8_t* out_status) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rc = raw_write_reg(c, addr, value, out_status);
  if (rc != 0) return rc;

  setErr(c, "");
  apply_limit_safety_always(h);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_read_reg(TMC5160_Handle h, uint8_t addr, uint32_t* out_value, uint8_t* out_status) {
  if (!h || !out_value) {
    if (h) return stop_on_error((Context*)h, TMC5160_ERR_INVALID_ARG, "read_reg", "read_reg: invalid arg");
    return TMC5160_ERR_INVALID_ARG;
  }
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rc = raw_read_reg(c, addr, out_value, out_status);
  if (rc != 0) return rc;

  setErr(c, "");
  apply_limit_safety_always(h);
  return TMC5160_OK;
}

// ============================================================================
// Debug dump
// ============================================================================
TMC_API int32_t tmc5160_dump_inputs(TMC5160_Handle h,
                                   uint32_t* out_ioin,
                                   uint32_t* out_rampstat,
                                   uint32_t* out_sw_mode)
{
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  if (out_ioin)     *out_ioin = 0;
  if (out_rampstat) *out_rampstat = 0;
  if (out_sw_mode)  *out_sw_mode = 0;

  uint32_t v = 0;
  int32_t rc = rd_u32(h, REG_IOIN, &v);
  if (rc != 0) return stop_on_error(c, rc, "dump_inputs", "dump_inputs: read IOIN failed");
  if (out_ioin) *out_ioin = v;

  rc = rd_u32(h, REG_RAMPSTAT, &v);
  if (rc != 0) return stop_on_error(c, rc, "dump_inputs", "dump_inputs: read RAMPSTAT failed");
  if (out_rampstat) *out_rampstat = v;

  rc = rd_u32(h, REG_SW_MODE, &v);
  if (rc != 0) return stop_on_error(c, rc, "dump_inputs", "dump_inputs: read SW_MODE failed");
  if (out_sw_mode) *out_sw_mode = v;

  return TMC5160_OK;
}

// ============================================================================
// Export: clear fault latch
// ============================================================================
TMC_API int32_t tmc5160_clear_fault(TMC5160_Handle h) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  // restore driver outputs if we panicked
  driver_restore_outputs_best_effort(c);

  c->fault_latched = false;
  c->fault_reason.clear();
  c->last_event_bits = 0;
  c->last_stop_status = 0;
  c->l_status_cnt = c->r_status_cnt = 0;
  setErr(c, "");

  // keep in safe stopped state
  (void)cancel_and_hold_strict(c, 500, false);
  return TMC5160_OK;
}

// ============================================================================
// High-level motor API
// ============================================================================
static int mres_from_microsteps(int32_t microsteps) {
  switch (microsteps) {
    case 256: return 0; case 128: return 1; case 64: return 2; case 32: return 3;
    case 16:  return 4; case 8:   return 5; case 4:  return 6; case 2:  return 7;
    case 1:   return 8; default:  return -1;
  }
}

TMC_API int32_t tmc5160_setup_motor(TMC5160_Handle h,
                                   uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                                   int32_t microsteps,
                                   uint32_t chopconf_base)
{
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rej = rejectIfFaulted(c, "setup_motor");
  if (rej) return rej;

  int32_t rc = force_hold_stop(h, 2000);
  if (rc != 0) return stop_on_error(c, rc, "setup_motor", "setup_motor: pre-stop failed");

  int mres = mres_from_microsteps(microsteps);
  if (mres < 0) return stop_on_error(c, TMC5160_ERR_INVALID_ARG, "setup_motor", "setup_motor: invalid microsteps");

  if (chopconf_base == 0) chopconf_base = 0x000100C3;

  uint32_t chopconf = (chopconf_base & ~(0xFu << 24)) | ((uint32_t)mres << 24);
  uint32_t ihold_irun = (ihold & 0x1F) | ((irun & 0x1F) << 8) | ((iholddelay & 0x0F) << 16);

  uint8_t st = 0;
  if (tmc5160_write_reg(h, REG_GCONF, 0x00000000, &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "setup_motor", "setup_motor: write GCONF failed");
  if (tmc5160_write_reg(h, REG_CHOPCONF, chopconf, &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "setup_motor", "setup_motor: write CHOPCONF failed");
  if (tmc5160_write_reg(h, REG_IHOLD_IRUN, ihold_irun, &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "setup_motor", "setup_motor: write IHOLD_IRUN failed");
  if (tmc5160_write_reg(h, REG_TPOWERDOWN, 10, &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "setup_motor", "setup_motor: write TPOWERDOWN failed");

  (void)tmc5160_write_reg(h, REG_GSTAT, 0x00000007, &st);

  rc = force_hold_stop(h, 2000);
  if (rc != 0) return stop_on_error(c, rc, "setup_motor", "setup_motor: post-stop failed");

  rc = arm_hw_limits_for_normal_motion(h, c);
  if (rc != 0) return stop_on_error(c, rc, "setup_motor", "setup_motor: arm_hw_limits failed");

  setErr(c, "");
  apply_limit_safety_always(h);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_set_ramp_params(TMC5160_Handle h,
                                       uint32_t vstart, uint32_t a1, uint32_t v1,
                                       uint32_t amax,   uint32_t vmax,
                                       uint32_t dmax,   uint32_t d1,
                                       uint32_t vstop)
{
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rej = rejectIfFaulted(c, "set_ramp_params");
  if (rej) return rej;

  uint8_t st=0;
  if (tmc5160_write_reg(h, REG_VSTART, vstart, &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write VSTART failed");
  if (tmc5160_write_reg(h, REG_A1,     a1,     &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write A1 failed");
  if (tmc5160_write_reg(h, REG_V1,     v1,     &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write V1 failed");
  if (tmc5160_write_reg(h, REG_AMAX,   amax,   &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write AMAX failed");
  if (tmc5160_write_reg(h, REG_VMAX,   vmax,   &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write VMAX failed");
  if (tmc5160_write_reg(h, REG_DMAX,   dmax,   &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write DMAX failed");
  if (tmc5160_write_reg(h, REG_D1,     d1,     &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write D1 failed");
  if (tmc5160_write_reg(h, REG_VSTOP,  vstop,  &st) != 0) return stop_on_error(c, TMC5160_ERR_IO, "set_ramp_params", "write VSTOP failed");

  int32_t rc = arm_hw_limits_for_normal_motion(h, c);
  if (rc != 0) return stop_on_error(c, rc, "set_ramp_params", "arm_hw_limits failed");

  apply_limit_safety_always(h);

  c->cached_vmax = vmax;
  c->have_cached_vmax = (vmax != 0);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_set_pos(TMC5160_Handle h, int32_t pos) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rej = rejectIfFaulted(c, "set_pos");
  if (rej) return rej;

  uint8_t st=0;
  if (tmc5160_write_reg(h, REG_XACTUAL, (uint32_t)pos, &st) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "set_pos", "write XACTUAL failed");

  int32_t rc = arm_hw_limits_for_normal_motion(h, c);
  if (rc != 0) return stop_on_error(c, rc, "set_pos", "arm_hw_limits failed");

  apply_limit_safety_always(h);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_get_pos(TMC5160_Handle h, int32_t* out_pos) {
  if (!h || !out_pos) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  uint32_t v=0; uint8_t st=0;
  if (tmc5160_read_reg(h, REG_XACTUAL, &v, &st) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "get_pos", "read XACTUAL failed");

  *out_pos = (int32_t)v;
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_get_vel(TMC5160_Handle h, int32_t* out_vel) {
  if (!h || !out_vel) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  int32_t v = 0;
  if (rd_s24(h, REG_VACTUAL, &v) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "get_vel", "read VACTUAL failed");

  *out_vel = v;
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_move_to(TMC5160_Handle h, int32_t pos) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rej = rejectIfFaulted(c, "move_to");
  if (rej) return rej;

  int32_t rc = arm_hw_limits_for_normal_motion(h, c);
  if (rc != 0) return stop_on_error(c, rc, "move_to", "arm_hw_limits failed");

  begin_motion_epoch(h, c);
  apply_limit_safety_always(h);
  if (c->fault_latched) return rejectIfFaulted(c, "move_to");

  rc = ensure_vmax_restored_for_position(h, c);
  if (rc != 0) return rc;

  rc = wr_u32(h, REG_RAMPMODE, RM_POSITION);
  if (rc != 0) return stop_on_error(c, rc, "move_to", "write RAMPMODE failed");

  rc = wr_u32(h, REG_VMAX, c->cached_vmax);
  if (rc != 0) return stop_on_error(c, rc, "move_to", "restore VMAX failed");

  rc = wr_u32(h, REG_XTARGET, (uint32_t)pos);
  if (rc != 0) return stop_on_error(c, rc, "move_to", "write XTARGET failed");

  apply_limit_safety_always(h);
  if (c->fault_latched) return rejectIfFaulted(c, "move_to");

  setErr(c, "");
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_move_by(TMC5160_Handle h, int32_t delta) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  int32_t cur = 0;
  int32_t rc = tmc5160_get_pos(h, &cur);
  if (rc != 0) return rc;
  return tmc5160_move_to(h, cur + delta);
}

TMC_API int32_t tmc5160_rotate(TMC5160_Handle h, int32_t velocity) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  int32_t rej = rejectIfFaulted(c, "rotate");
  if (rej) return rej;

  int32_t rc = arm_hw_limits_for_normal_motion(h, c);
  if (rc != 0) return stop_on_error(c, rc, "rotate", "arm_hw_limits failed");

  begin_motion_epoch(h, c);
  apply_limit_safety_always(h);
  if (c->fault_latched) return rejectIfFaulted(c, "rotate");

  rc = start_rotate_with_vmax(h, velocity);
  if (rc != 0) return stop_on_error(c, rc, "rotate", "start_rotate failed");

  apply_limit_safety_always(h);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_stop(TMC5160_Handle h) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  const bool ok = cancel_and_hold_strict(c, 2000, true);
  if (!ok) {
    // ここまで来たら kill 済みの可能性が高い。fault を立ててブロックする。
    setErr(c, "stop: failed to reach standstill -> panic kill applied");
    latch_fault(c, "stop panic");
    return TMC5160_ERR_TIMEOUT;
  }
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_is_pos_reached(TMC5160_Handle h, int* out_yes) {
  if (!h || !out_yes) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  uint32_t v=0; uint8_t st=0;
  if (tmc5160_read_reg(h, REG_RAMPSTAT, &v, &st) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "is_pos_reached", "read RAMPSTAT failed");

  *out_yes = ((v >> 9) & 1u) ? 1 : 0;
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_wait_pos_reached(TMC5160_Handle h, uint32_t timeout_ms) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  auto t0 = std::chrono::steady_clock::now();
  while (true) {
    apply_limit_safety_always(h);
    if (c->fault_latched) return rejectIfFaulted(c, "wait_pos_reached");

    uint32_t rs = 0;
    int32_t rc = rd_u32(h, REG_RAMPSTAT, &rs);
    if (rc != 0) return rc;

    if (ramp_pos_reached(rs)) return TMC5160_OK;

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) return stop_on_error(c, TMC5160_ERR_TIMEOUT, "wait_pos_reached", "wait_pos_reached: timeout");

    tiny_yield();
  }
}

// ============================================================================
// Encoder
// ============================================================================
TMC_API int32_t tmc5160_setup_encoder(TMC5160_Handle h, uint32_t encmode) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  int32_t rej = rejectIfFaulted(c, "setup_encoder");
  if (rej) return rej;

  uint8_t st=0;
  if (tmc5160_write_reg(h, REG_ENCMODE, encmode, &st) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "setup_encoder", "write ENCMODE failed");

  apply_limit_safety_always(h);
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_get_enc_pos(TMC5160_Handle h, int32_t* out_pos) {
  if (!h || !out_pos) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  uint32_t v=0; uint8_t st=0;
  if (tmc5160_read_reg(h, REG_X_ENC, &v, &st) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "get_enc_pos", "read X_ENC failed");

  *out_pos = (int32_t)v;
  return TMC5160_OK;
}

TMC_API int32_t tmc5160_set_enc_pos(TMC5160_Handle h, int32_t pos) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  int32_t rej = rejectIfFaulted(c, "set_enc_pos");
  if (rej) return rej;

  uint8_t st=0;
  if (tmc5160_write_reg(h, REG_X_ENC, (uint32_t)pos, &st) != 0)
    return stop_on_error(c, TMC5160_ERR_IO, "set_enc_pos", "write X_ENC failed");

  apply_limit_safety_always(h);
  return TMC5160_OK;
}

// ============================================================================
// Ref pins
// ============================================================================
TMC_API int32_t tmc5160_get_ref_pins(TMC5160_Handle h, int* out_refl, int* out_refr) {
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;

  uint32_t v = 0;
  int32_t rc = rd_u32(h, REG_IOIN, &v);
  if (rc != 0) return stop_on_error(c, rc, "get_ref_pins", "read IOIN failed");

  if (out_refl) *out_refl = (v & (1u << 0)) ? 1 : 0;
  if (out_refr) *out_refr = (v & (1u << 1)) ? 1 : 0;
  return TMC5160_OK;
}

// 片側のリミットだけを見る待ち（OFF安定→ON安定 or EVENT立上り）
// ★assume_armed=true の場合、release確認(arming)をスキップして「押下待ち」する
static int32_t wait_homing_switch(TMC5160_Handle h,
                                 bool target_right,
                                 bool active_low,
                                 uint32_t timeout_ms,
                                 uint32_t* out_final_rs,
                                 bool assume_armed)
{
  auto* c = ctx_from_handle(h);
  if (!c) return TMC5160_ERR_INVALID_ARG;

  auto t0 = std::chrono::steady_clock::now();
  if (out_final_rs) *out_final_rs = 0;

  const int ARM_RELEASE_REQ  = 10;
  const int PRESS_STABLE_REQ = 10;
  const int OTHER_STABLE_REQ = 10;

  bool started = false;

  bool armed   = assume_armed;
  int  rel_cnt = assume_armed ? ARM_RELEASE_REQ : 0;

  int  prs_cnt = 0;
  int  oth_cnt = 0;

  bool oth_armed = false;
  int  oth_rel_cnt = 0;
  const int OTH_RELEASE_REQ = 10;

  bool ev_armed = false;
  uint32_t last_ev = 0;

  const uint32_t tgt_st = target_right ? RS_STATUS_STOP_R : RS_STATUS_STOP_L;
  const uint32_t oth_st = target_right ? RS_STATUS_STOP_L : RS_STATUS_STOP_R;
  const uint32_t tgt_ev = target_right ? RS_EVENT_STOP_R  : RS_EVENT_STOP_L;
  const uint32_t oth_ev = target_right ? RS_EVENT_STOP_L  : RS_EVENT_STOP_R;

  while (true) {
    if (c->fault_latched) return rejectIfFaulted(c, "home(wait_switch)");

    uint32_t rs = 0;
    int32_t  rc = rd_u32(h, REG_RAMPSTAT, &rs);
    if (rc != 0) return rc;

    const uint32_t ev_cur = rs & (RS_EVENT_STOP_L | RS_EVENT_STOP_R);
    uint32_t ev_rise = 0;

    if (!started) {
      last_ev = ev_cur;
    } else {
      if (!ev_armed) {
        last_ev = ev_cur;
        ev_armed = true;
      } else {
        ev_rise = ev_cur & ~last_ev;
        last_ev = ev_cur;
      }
    }

    uint32_t ioin = 0;
    rc = rd_u32(h, REG_IOIN, &ioin);
    if (rc != 0) return rc;

    int32_t vact = 0;
    rc = rd_s24(h, REG_VACTUAL, &vact);
    if (rc != 0) return rc;

    if (!started) {
      if (std::abs(vact) > 1) started = true;
    }

    const bool target_pressed = home_pressed(ioin,  target_right, active_low);
    const bool other_pressed  = home_pressed(ioin, !target_right, active_low);

    const bool tgt_status = (rs & tgt_st) != 0;
    const bool oth_status = (rs & oth_st) != 0;

    const bool tgt_pressed_ok  = target_pressed && tgt_status;
    const bool tgt_released_ok = (!target_pressed) && (!tgt_status);
    const bool oth_pressed_ok  = other_pressed && oth_status;

    if (started) {
      const bool oth_released_ok = (!other_pressed) && (!oth_status);

      if (!oth_armed) {
        if (oth_released_ok) {
          if (++oth_rel_cnt >= OTH_RELEASE_REQ) oth_armed = true;
        } else {
          oth_rel_cnt = 0;
        }
        oth_cnt = 0;
      } else {
        if (oth_pressed_ok) {
          if (++oth_cnt >= OTHER_STABLE_REQ) {
            if (out_final_rs) *out_final_rs = rs;
            return TMC5160_ERR_IO;
          }
        } else {
          oth_cnt = 0;
        }

        if (armed && (ev_rise & oth_ev) && oth_status) {
          if (out_final_rs) *out_final_rs = rs;
          return TMC5160_ERR_IO;
        }
      }

      if (!armed) {
        if (tgt_released_ok) {
          if (++rel_cnt >= ARM_RELEASE_REQ) armed = true;
        } else rel_cnt = 0;
      } else {
        if ((ev_rise & tgt_ev) && tgt_status) {
          if (out_final_rs) *out_final_rs = rs;
          return TMC5160_OK;
        }
        if (tgt_pressed_ok) {
          if (++prs_cnt >= PRESS_STABLE_REQ) {
            if (out_final_rs) *out_final_rs = rs;
            return TMC5160_OK;
          }
        } else prs_cnt = 0;
      }
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) {
      // ★timeout = 即停止（戻り値はtimeoutのまま）
      best_effort_cancel_and_hold(c);
      return TMC5160_ERR_TIMEOUT;
    }

    tiny_yield();
  }
}

// ============================================================================
// Homing（RIGHT/LEFT 対称で確実に動く版）
// 期待動作:
// RIGHT: 右へ→R押下で反転(左へ)→R離れたら反転(右へ)→R再押下で停止完了
// LEFT : 左へ→L押下で反転(右へ)→L離れたら反転(左へ)→L再押下で停止完了
// backoff_ustep 到達で即停止＋エラー
// ============================================================================
TMC_API int32_t tmc5160_home(TMC5160_Handle h,
                            int32_t dir,
                            uint32_t flags,
                            int32_t v_fast,
                            int32_t v_slow,
                            int32_t backoff_ustep,
                            uint32_t timeout_ms_each,
                            int32_t home_pos_ustep,
                            int32_t* out_latched_ustep)
{
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);
  if (dir == 0) return stop_on_error(c, TMC5160_ERR_INVALID_ARG, "home", "home: dir=0");

  int32_t rc = TMC5160_OK;

  uint32_t sw_old    = 0;
  uint32_t vstop_old = 0;
  bool have_sw_old = false;
  bool have_vstop_old = false;

  uint32_t xl_before = 0;
  std::string err_snapshot;

  int32_t rej = rejectIfFaulted(c, "home");
  if (rej) return rej;

  c->homing_active = true;
  if (out_latched_ustep) *out_latched_ustep = 0;

  // sanitize
  if (v_fast < 0) v_fast = -v_fast;
  if (v_slow < 0) v_slow = -v_slow;
  if (v_fast == 0) { rc = TMC5160_ERR_INVALID_ARG; setErr(c, "home: v_fast=0"); best_effort_cancel_and_hold(c); goto RESTORE; }
  if (v_slow == 0) v_slow = v_fast;
  if (backoff_ustep < 0) backoff_ustep = 0;

  const bool target_right = is_right_dir(dir);
  const bool use_latch   = (flags & TMC_HOME_USE_LATCH) != 0;
  const bool softstop    = (flags & TMC_HOME_SOFTSTOP) != 0;

  // polarity flags -> SW_MODE へ反映（最終的な判定も SW_MODE に合わせる）
  bool active_low = (flags & TMC_HOME_ACTIVE_LOW) != 0;
  const bool active_high = (flags & TMC_HOME_ACTIVE_HIGH) != 0;
  if (!active_low && !active_high) active_low = true;

  // まず完全停止で開始
  rc = force_hold_stop(h, timeout_ms_each);
  if (rc != 0) { setErr(c, "home: pre-stop timeout"); best_effort_cancel_and_hold(c); goto RESTORE; }

  // save old
  rc = rd_u32(h, REG_SW_MODE, &sw_old);
  if (rc != 0) { setErr(c, "home: read SW_MODE failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
  have_sw_old = true;

  rc = rd_u32(h, REG_VSTOP, &vstop_old);
  if (rc != 0) { setErr(c, "home: read VSTOP failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
  have_vstop_old = true;

  // build homing SW_MODE base (both stop enabled, polarity set, latch optional, softstop optional)
  uint32_t sw_base = sw_old;
  sw_base &= ~(SW_STOP_L_EN | SW_STOP_R_EN | SW_POL_STOP_L | SW_POL_STOP_R |
               SW_LATCH_L_ACTIVE | SW_LATCH_R_ACTIVE | SW_EN_SOFTSTOP);

  if (active_low) sw_base |= (SW_POL_STOP_L | SW_POL_STOP_R);
  if (use_latch)  sw_base |= (target_right ? SW_LATCH_R_ACTIVE : SW_LATCH_L_ACTIVE);
  if (softstop)   sw_base |= SW_EN_SOFTSTOP;
  sw_base |= (SW_STOP_L_EN | SW_STOP_R_EN);

  // 2nd approach は必ずハードストップ（softstop無し + VSTOP=0）
  const uint32_t sw_hard = (sw_base & ~SW_EN_SOFTSTOP);

  // apply base
  rc = wr_u32(h, REG_SW_MODE, sw_base);
  if (rc != 0) { setErr(c, "home: write SW_MODE failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
  (void)clear_rampstat_events(h);

  // --------------------------------------------------------------------------
  // Phase0: 初期押下なら逃がす（両方押下は即エラー）
  // --------------------------------------------------------------------------
  {
    uint32_t ioin = 0;
    rc = rd_u32(h, REG_IOIN, &ioin);
    if (rc != 0) { setErr(c, "home: read IOIN failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    const bool tgt_pressed = home_pressed(ioin,  target_right, active_low);
    const bool oth_pressed = home_pressed(ioin, !target_right, active_low);

    if (tgt_pressed && oth_pressed) {
      setErr(c, "home: both limits pressed at start");
      best_effort_cancel_and_hold(c);
      rc = TMC5160_ERR_IO;
      goto RESTORE;
    }

    if (tgt_pressed) {
      // 押されている側の STOP だけ無効化して、逆へ逃がす
      uint32_t sw_esc = sw_base;
      if (target_right) sw_esc &= ~SW_STOP_R_EN;
      else              sw_esc &= ~SW_STOP_L_EN;
      sw_esc &= ~SW_EN_SOFTSTOP;

      rc = wr_u32(h, REG_SW_MODE, sw_esc);
      if (rc != 0) { setErr(c, "home: set SW_MODE(escape) failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
      (void)clear_rampstat_events(h);

      const int32_t vel_away = target_right ? -v_slow : +v_slow;
      rc = start_rotate_with_vmax(h, vel_away);
      if (rc != 0) { setErr(c, "home: escape rotate failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

      // 両方OFF安定待ち
      auto t0 = std::chrono::steady_clock::now();
      int stable = 0;
      const int STABLE_REQ = 40;

      while (true) {
        uint32_t io = 0;
        rc = rd_u32(h, REG_IOIN, &io);
        if (rc != 0) { setErr(c, "home: escape read IOIN failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

        const bool tp = home_pressed(io, target_right, active_low);

        if (!tp) {
          if (++stable >= STABLE_REQ) break;
        } else stable = 0;

        auto now = std::chrono::steady_clock::now();
        uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (ms >= timeout_ms_each) {
          setErr(c, "home: escape release timeout");
          best_effort_cancel_and_hold(c);
          rc = TMC5160_ERR_TIMEOUT;
          goto RESTORE;
        }
        tiny_yield();
      }

      (void)force_hold_stop(h, timeout_ms_each);
      (void)wr_u32(h, REG_SW_MODE, sw_base);
      (void)clear_rampstat_events(h);
    }
  }

  // --------------------------------------------------------------------------
  // Phase1: 1st approach（fast）: target を押したら停止確定
  // --------------------------------------------------------------------------
  {
    // ここも「確実に再開できる」ように、フェーズ開始前に一度停止→設定→イベントクリア
    (void)force_hold_stop(h, timeout_ms_each);

    rc = wr_u32(h, REG_SW_MODE, sw_base);
    if (rc != 0) { setErr(c, "home: set SW_MODE(base) failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
    (void)clear_rampstat_events(h);

    const int32_t vel_to_target = target_right ? +v_fast : -v_fast;
    rc = start_rotate_with_vmax(h, vel_to_target);
    if (rc != 0) { setErr(c, "home: start fast failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    uint32_t rs1 = 0;
    rc = wait_homing_switch(h, target_right, active_low, timeout_ms_each, &rs1, false);

    if (rc != 0) {
      setErr(c, (rc == TMC5160_ERR_TIMEOUT) ? "home: 1st press timeout" : "home: 1st press failed (other hit?)");
      best_effort_cancel_and_hold(c);
      goto RESTORE;
    }

    // ★重要：RIGHT不具合潰しポイント
    // STOPイベントで止まった直後は内部状態が残ることがあるので、必ず hold stop で確定させる
    rc = force_hold_stop(h, timeout_ms_each);
    if (rc != 0) { setErr(c, "home: stop after 1st press timeout"); best_effort_cancel_and_hold(c); goto RESTORE; }
  }

  // --------------------------------------------------------------------------
  // Phase2: backoff（slow, target STOP無効）: target が離れたら次へ
  //   backoff_ustep 超過で即停止＋エラー
  // --------------------------------------------------------------------------
  {
    // フェーズ設定
    uint32_t sw_backoff = sw_base;
    sw_backoff &= ~SW_EN_SOFTSTOP;
    sw_backoff &= ~(SW_LATCH_L_ACTIVE | SW_LATCH_R_ACTIVE);

    // ★重要：押下中でも離れる方向へ動けるよう、ターゲット側STOPだけ無効化
    if (target_right) sw_backoff &= ~SW_STOP_R_EN;
    else              sw_backoff &= ~SW_STOP_L_EN;

    rc = wr_u32(h, REG_SW_MODE, sw_backoff);
    if (rc != 0) { setErr(c, "home: set SW_MODE(backoff) failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
    (void)clear_rampstat_events(h);

    // backoff 開始位置
    int32_t x0 = 0;
    rc = rd_s32(h, REG_XACTUAL, &x0);
    if (rc != 0) { setErr(c, "home: backoff read XACTUAL failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    const int32_t vel_backoff = target_right ? -v_slow : +v_slow;
    rc = start_rotate_with_vmax(h, vel_backoff);
    if (rc != 0) { setErr(c, "home: start backoff failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    auto t0 = std::chrono::steady_clock::now();
    int rel_stable = 0;
    const int RELEASE_STABLE_REQ = 5;   // 3〜10 で調整可

    int opp_stable = 0;
    const int OPP_STABLE_REQ = 5;

    while (true) {
      if (c->fault_latched) { rc = rejectIfFaulted(c, "home: backoff"); best_effort_cancel_and_hold(c); goto RESTORE; }

      uint32_t ioin = 0;
      rc = rd_u32(h, REG_IOIN, &ioin);
      if (rc != 0) { setErr(c, "home: backoff read IOIN failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

      const bool tgt_pressed = home_pressed(ioin, target_right, active_low);
      const bool oth_pressed = home_pressed(ioin, !target_right, active_low);

      // opposite hit => error
      if (oth_pressed) {
        if (++opp_stable >= OPP_STABLE_REQ) {
          setErr(c, "home: backoff hit opposite limit");
          best_effort_cancel_and_hold(c);
          rc = TMC5160_ERR_IO;
          goto RESTORE;
        }
      } else opp_stable = 0;

      // target released?
      if (!tgt_pressed) {
        if (++rel_stable >= RELEASE_STABLE_REQ) break;   // ★ここでwhileを抜けて次フェーズへ
      } else {
        rel_stable = 0;
      }

      // distance guard
      if (backoff_ustep > 0) {
        int32_t x = 0;
        if (rd_s32(h, REG_XACTUAL, &x) == 0) {
          int32_t d = x - x0; if (d < 0) d = -d;
          if (d >= backoff_ustep) {
            setErr(c, "home: backoff exceeded distance (switch not releasing?)");
            best_effort_cancel_and_hold(c);
            rc = TMC5160_ERR_TIMEOUT;
            goto RESTORE;
          }
        }
      }

      auto now = std::chrono::steady_clock::now();
      uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
      if (ms >= timeout_ms_each) {
        setErr(c, "home: release timeout");
        best_effort_cancel_and_hold(c);
        rc = TMC5160_ERR_TIMEOUT;
        goto RESTORE;
      }

      tiny_yield();
    }

    // -----------------------------
    // release 確定 → 2nd approach（再押下で停止完了）
    // 方針：
    //  - 2nd approach では target STOP を有効に戻し、2回目押下で HW 停止を確実に掛ける
    //  - ただし STOP 状態残りで再開不能にならないよう、STOP有効化後に「targetが離れている」を短時間再確認
    // -----------------------------

    // 2nd approach 用 SW_MODE（hard相当・softstopなし）
    // ※sw_hard は sw_base 由来なので use_latch なら latch ビットも入っている
    uint32_t sw_app2 = sw_hard;
    sw_app2 &= ~SW_EN_SOFTSTOP; // 念のため

    // ★重要：ここでは target STOP を無効にしない（2回目押下で止めるため）
    // （以前の“開始不能”対策は、下の「離れている再確認」で潰す）

    rc = wr_u32(h, REG_SW_MODE, sw_app2);
    if (rc != 0) { setErr(c, "home: set SW_MODE(app2) failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    rc = wr_u32(h, REG_VSTOP, 0);
    if (rc != 0) { setErr(c, "home: set VSTOP=0 failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    (void)clear_rampstat_events(h);

    // STOP 有効化後に、target が確実に離れていることを再確認（残留STOPで開始不能を根絶）
    {
      auto tchk = std::chrono::steady_clock::now();
      int stable = 0;
      const int STABLE_REQ = 10; // 10ms
      const uint32_t tgt_st = target_right ? RS_STATUS_STOP_R : RS_STATUS_STOP_L;

      while (true) {
        uint32_t io = 0, rs_chk = 0;
        rc = rd_u32(h, REG_IOIN, &io);
        if (rc != 0) { setErr(c, "home: app2 precheck read IOIN failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
        rc = rd_u32(h, REG_RAMPSTAT, &rs_chk);
        if (rc != 0) { setErr(c, "home: app2 precheck read RAMPSTAT failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

        const bool tgt_pressed = home_pressed(io, target_right, active_low);
        const bool tgt_status  = (rs_chk & tgt_st) != 0;

        if (!tgt_pressed && !tgt_status) {
          if (++stable >= STABLE_REQ) break;
        } else {
          stable = 0;
        }

        auto now = std::chrono::steady_clock::now();
        uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - tchk).count();
        if (ms >= 200) { // 0.2s で十分
          setErr(c, "home: app2 precheck (switch not released)");
          best_effort_cancel_and_hold(c);
          rc = TMC5160_ERR_IO;
          goto RESTORE;
        }
        tiny_yield();
      }
    }

    // 2nd approach start（反転）
    const int32_t vel_approach2 = target_right ? +v_slow : -v_slow;
    rc = start_rotate_with_vmax(h, vel_approach2);
    if (rc != 0) { setErr(c, "home: start 2nd approach failed"); best_effort_cancel_and_hold(c); goto RESTORE; }

    if (use_latch) (void)rd_u32(h, REG_XLATCH, &xl_before);

    // 2nd press 待ち（release確認済みなので assume_armed=true）
    uint32_t rs2 = 0;
    rc = wait_homing_switch(h, target_right, active_low, timeout_ms_each, &rs2, true);
    if (rc != 0) {
      setErr(c, (rc == TMC5160_ERR_TIMEOUT) ? "home: 2nd press timeout" : "home: 2nd press failed (other hit?)");
      best_effort_cancel_and_hold(c);
      goto RESTORE;
    }

    // HW stop 後も確実に停止状態へ
    rc = force_hold_stop(h, timeout_ms_each);
    if (rc != 0) { setErr(c, "home: stop after 2nd press timeout"); best_effort_cancel_and_hold(c); goto RESTORE; }

    (void)force_hold_stop(h, timeout_ms_each);

    // latch / pos
    int32_t latch_pos = 0;
    if (use_latch) {
      uint32_t xl_after = 0;
      (void)rd_u32(h, REG_XLATCH, &xl_after);
      if (xl_after != xl_before) latch_pos = (int32_t)xl_after;
      else {
        int32_t trc = tmc5160_get_pos(h, &latch_pos);
        if (trc != 0) { rc = trc; setErr(c, "home: read pos after 2nd press failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
      }
    } else {
      int32_t trc = tmc5160_get_pos(h, &latch_pos);
      if (trc != 0) { rc = trc; setErr(c, "home: read pos after 2nd press failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
    }

    if (out_latched_ustep) *out_latched_ustep = latch_pos;

    rc = tmc5160_set_pos(h, home_pos_ustep);
    if (rc != 0) { setErr(c, "home: set_pos failed"); best_effort_cancel_and_hold(c); goto RESTORE; }
    (void)wr_u32(h, REG_XTARGET, (uint32_t)home_pos_ustep);

    rc = TMC5160_OK;
  }

RESTORE:
  if (rc != 0) err_snapshot = c ? c->last_err : std::string();

  if (rc != 0) {
    latch_fault(c, "home error");
    best_effort_cancel_and_hold(c);
  }

  // restore regs
  if (have_vstop_old) (void)wr_u32(h, REG_VSTOP, vstop_old);
  if (have_sw_old)    (void)wr_u32(h, REG_SW_MODE, sw_old);

  apply_limit_safety_always(h);
  c->homing_active = false;

  if (rc != 0) {
    if (!err_snapshot.empty()) setErr(c, err_snapshot);
    else setErr(c, (rc == TMC5160_ERR_TIMEOUT) ? "home: timeout" : "home: failed");
    best_effort_cancel_and_hold(c);
    return rc;
  }

  // normal motion limits re-arm
  (void)arm_hw_limits_for_normal_motion(h, c);
  return TMC5160_OK;
}

// ============================================================================
// Move until limit (LEFT/RIGHT) - returns OK on limit hit (no fault latch)
// - Hardware stop via SW_MODE STOP inputs (hard stop, no softstop)
// - Also issues SW cancel+HOLD to guarantee stop
// ============================================================================
TMC_API int32_t tmc5160_move_to_limit(TMC5160_Handle h,
                                      int32_t dir,
                                      int32_t velocity,
                                      uint32_t timeout_ms,
                                      int32_t* out_hit_dir)
{
  if (!h) return TMC5160_ERR_INVALID_ARG;
  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);

  if (out_hit_dir) *out_hit_dir = 0;

  if (dir == 0) {
    return stop_on_error(c, TMC5160_ERR_INVALID_ARG, "move_to_limit", "move_to_limit: dir=0");
  }

  if (velocity < 0) velocity = -velocity;
  if (velocity == 0) {
    return stop_on_error(c, TMC5160_ERR_INVALID_ARG, "move_to_limit", "move_to_limit: velocity=0");
  }

  int32_t rej = rejectIfFaulted(c, "move_to_limit");
  if (rej) return rej;

  const bool target_right = (dir > 0);

  uint32_t sw_old = 0, vstop_old = 0;
  bool have_sw_old = false, have_vstop_old = false;

  const bool homing_prev = c->homing_active;
  c->homing_active = true; // ★limit hit を fault latch しないため（apply_limit_safety_always対策）

  int32_t rc = TMC5160_OK;

  // まず停止して開始
  rc = force_hold_stop(h, 2000);
  if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: pre-stop failed"); goto RESTORE_ERR; }

  // Save current settings to restore
  rc = rd_u32(h, REG_SW_MODE, &sw_old);
  if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: read SW_MODE failed"); goto RESTORE_ERR; }
  have_sw_old = true;

  rc = rd_u32(h, REG_VSTOP, &vstop_old);
  if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: read VSTOP failed"); goto RESTORE_ERR; }
  have_vstop_old = true;

  // Build SW_MODE for hard limit stop (both enabled, polarity from ctx, softstop OFF)
  uint32_t sw = sw_old;

  // enable stop inputs
  sw |= (SW_STOP_L_EN | SW_STOP_R_EN);

  // polarity
  if (c->limits_active_low) sw |= (SW_POL_STOP_L | SW_POL_STOP_R);
  else                     sw &= ~(SW_POL_STOP_L | SW_POL_STOP_R);

  // hard stop (no softstop)
  sw &= ~SW_EN_SOFTSTOP;

  // avoid latch side-effects while doing limit seek
  sw &= ~(SW_LATCH_L_ACTIVE | SW_LATCH_R_ACTIVE);

  rc = wr_u32(h, REG_SW_MODE, sw);
  if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: write SW_MODE failed"); goto RESTORE_ERR; }

  // ensure hard stop behavior
  rc = wr_u32(h, REG_VSTOP, 0);
  if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: write VSTOP failed"); goto RESTORE_ERR; }

  (void)clear_rampstat_events(h);
  begin_motion_epoch(h, c);

  // If already pressed at start -> just stop and return OK (no error)
  {
    uint32_t io = 0;
    rc = rd_u32(h, REG_IOIN, &io);
    if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: read IOIN failed"); goto RESTORE_ERR; }

    const bool lp = pressed_from_ioin_sw(io, false, sw);
    const bool rp = pressed_from_ioin_sw(io, true,  sw);

    if (lp || rp) {
      if (out_hit_dir) {
        if (rp && !lp) *out_hit_dir = +1;
        else if (lp && !rp) *out_hit_dir = -1;
        else *out_hit_dir = 0;
      }
      (void)force_hold_stop(h, 2000);
      setErr(c, "");
      rc = TMC5160_OK;
      goto RESTORE_OK;
    }
  }

  // Start moving toward target
  {
    const int32_t vel = target_right ? +velocity : -velocity;
    rc = start_rotate_with_vmax(h, vel);
    if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: start_rotate failed"); goto RESTORE_ERR; }
  }

  // Poll until limit hit (target or opposite). Limit hit is NOT an error.
  {
    auto t0 = std::chrono::steady_clock::now();

    int tgt_cnt = 0;
    int oth_cnt = 0;
    const int STABLE_REQ = 2; // ~2ms安定で確定（必要なら増やしてOK）

    const uint32_t tgt_st = target_right ? RS_STATUS_STOP_R : RS_STATUS_STOP_L;
    const uint32_t oth_st = target_right ? RS_STATUS_STOP_L : RS_STATUS_STOP_R;
    const uint32_t tgt_ev = target_right ? RS_EVENT_STOP_R  : RS_EVENT_STOP_L;
    const uint32_t oth_ev = target_right ? RS_EVENT_STOP_L  : RS_EVENT_STOP_R;

    uint32_t last_ev = 0;

    while (true) {
      uint32_t rs = 0;
      rc = rd_u32(h, REG_RAMPSTAT, &rs);
      if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: read RAMPSTAT failed"); goto RESTORE_ERR; }

      uint32_t io = 0;
      rc = rd_u32(h, REG_IOIN, &io);
      if (rc != 0) { rc = stop_on_error(c, rc, "move_to_limit", "move_to_limit: read IOIN failed"); goto RESTORE_ERR; }

      const bool tgt_pressed = pressed_from_ioin_sw(io,  target_right, sw);
      const bool oth_pressed = pressed_from_ioin_sw(io, !target_right, sw);

      const bool tgt_status = (rs & tgt_st) != 0;
      const bool oth_status = (rs & oth_st) != 0;

      const uint32_t ev_cur  = rs & (RS_EVENT_STOP_L | RS_EVENT_STOP_R);
      const uint32_t ev_rise = ev_cur & ~last_ev;
      last_ev = ev_cur;

      // event rising -> treat as immediate stable hit
      if ((ev_rise & tgt_ev) && tgt_status) tgt_cnt = STABLE_REQ;
      if ((ev_rise & oth_ev) && oth_status) oth_cnt = STABLE_REQ;

      // stable pressed + status
      if (tgt_pressed && tgt_status) { if (tgt_cnt < STABLE_REQ) tgt_cnt++; } else tgt_cnt = 0;
      if (oth_pressed && oth_status) { if (oth_cnt < STABLE_REQ) oth_cnt++; } else oth_cnt = 0;

      if (tgt_cnt >= STABLE_REQ || oth_cnt >= STABLE_REQ) {
        int32_t hit_dir = 0;
        if (tgt_cnt >= STABLE_REQ) hit_dir = target_right ? +1 : -1;
        else                       hit_dir = target_right ? -1 : +1;

        if (out_hit_dir) *out_hit_dir = hit_dir;

        // HW stop should already be active, but also enforce SW cancel+HOLD for certainty
        best_effort_cancel_and_hold(c);
        (void)force_hold_stop(h, 2000);

        setErr(c, "");
        rc = TMC5160_OK;
        goto RESTORE_OK;
      }

      auto now = std::chrono::steady_clock::now();
      uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
      if (ms >= timeout_ms) {
        // timeout is an error (stop motor)
        setErr(c, "move_to_limit: timeout");
        best_effort_cancel_and_hold(c);
        rc = TMC5160_ERR_TIMEOUT;
        goto RESTORE_ERR;
      }

      tiny_yield();
    }
  }

RESTORE_ERR:
  // 失敗時は既存ポリシー通り fault latch
  if (rc != 0) {
    latch_fault(c, "move_to_limit error");
    best_effort_cancel_and_hold(c);
  }

RESTORE_OK:
  // restore registers
  if (have_vstop_old) (void)wr_u32(h, REG_VSTOP, vstop_old);
  if (have_sw_old)    (void)wr_u32(h, REG_SW_MODE, sw_old);

  c->homing_active = homing_prev;

  // re-arm normal motion limits
  (void)arm_hw_limits_for_normal_motion(h, c);

  // safety check
  apply_limit_safety_always(h);

  return rc;
}

// ============================================================================
// Move to many (sequence) - robust version (stop on any error, never "runaway")
// flags:
//  - TMC_SEQ_RELATIVE   : each entry is delta from current XACTUAL at the time of command
//  - TMC_SEQ_WAIT_XTOL  : wait pos-reached + VZERO after each move (recommended for reliability)
// out_done_count: number of completed moves (0..count)
// ============================================================================
static int32_t wait_vzero_with_safety(TMC5160_Handle h, uint32_t timeout_ms) {
  auto* c = ctx_from_handle(h);
  if (!c) return TMC5160_ERR_INVALID_ARG;

  auto t0 = std::chrono::steady_clock::now();
  uint32_t ctr = 0;

  while (true) {
    apply_limit_safety_always(h);
    if (c->fault_latched) return rejectIfFaulted(c, "wait_vzero");

    uint32_t rs = 0;
    int32_t rc = rd_u32(h, REG_RAMPSTAT, &rs);
    if (rc != 0) return rc;

    int32_t vact = 0;
    rc = rd_s24(h, REG_VACTUAL, &vact);
    if (rc != 0) return rc;

    if ((rs & RS_VZERO) || (std::abs(vact) <= 1)) return TMC5160_OK;

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) {
      // ★timeout = 即停止（戻り値はtimeoutのまま）
      best_effort_cancel_and_hold(c);
      return TMC5160_ERR_TIMEOUT;
    }

    poll_yield(ctr);
  }
}

// 目標位置の到達待ち（tol対応 + safety）
// - tol_ustep <= 0 の場合：RAMPSTATの position_reached(bit9) を使用
// - tol_ustep  > 0 の場合：|XACTUAL - target| <= tol を使用（より確実）
// どちらも fault/limit を監視して即停止できるよう apply_limit_safety_always を回す
static int32_t wait_pos_reached_tol_with_safety(TMC5160_Handle h,
                                               int32_t target,
                                               int32_t tol_ustep,
                                               uint32_t timeout_ms)
{
  auto* c = ctx_from_handle(h);
  if (!c) return TMC5160_ERR_INVALID_ARG;

  auto t0 = std::chrono::steady_clock::now();
  uint32_t ctr = 0;

  while (true) {
    apply_limit_safety_always(h);
    if (c->fault_latched) return rejectIfFaulted(c, "wait_pos_reached_tol");

    if (tol_ustep > 0) {
      int32_t x = 0;
      int32_t rc = rd_s32(h, REG_XACTUAL, &x);
      if (rc != 0) return rc;

      int32_t d = x - target;
      if (d < 0) d = -d;
      if (d <= tol_ustep) return TMC5160_OK;
    } else {
      uint32_t rs = 0;
      int32_t rc = rd_u32(h, REG_RAMPSTAT, &rs);
      if (rc != 0) return rc;
      if (ramp_pos_reached(rs)) return TMC5160_OK;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) {
      // ★timeout = 即停止（戻り値はtimeoutのまま）
      best_effort_cancel_and_hold(c);
      return TMC5160_ERR_TIMEOUT;
    }

    poll_yield(ctr);
  }
}

// dwell（停止中に待つだけ。待機中も safety を回す）
static void dwell_with_safety(TMC5160_Handle h, uint32_t dwell_ms) {
  if (dwell_ms == 0) return;
  auto* c = ctx_from_handle(h);
  if (!c) return;

  auto t0 = std::chrono::steady_clock::now();
  uint32_t ctr = 0;
  while (true) {
    apply_limit_safety_always(h);
    if (c->fault_latched) { best_effort_cancel_and_hold(c); return; }

    auto now = std::chrono::steady_clock::now();
    uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= dwell_ms) return;

    poll_yield(ctr);
  }
}

// ============================================================================
// Move to many (sequence) - header-matching version
// Header:
// TMC_API int32_t tmc5160_move_to_many(TMC5160_Handle h,
//                                     const int32_t* positions,
//                                     uint32_t count,
//                                     uint32_t dwell_ms_each,
//                                     uint32_t timeout_ms_each,
//                                     uint32_t flags,
//                                     int32_t tol_ustep,
//                                     uint32_t* out_done_count);
// ============================================================================
TMC_API int32_t tmc5160_move_to_many(TMC5160_Handle h,
                                    const int32_t* positions,
                                    uint32_t count,
                                    uint32_t dwell_ms_each,
                                    uint32_t timeout_ms_each,
                                    uint32_t flags,
                                    int32_t tol_ustep,
                                    uint32_t* out_done_count)
{
  if (!h) return TMC5160_ERR_INVALID_ARG;
  if (!positions && count) return TMC5160_ERR_INVALID_ARG;

  auto* c = (Context*)h;
  std::lock_guard<std::recursive_mutex> lk(c->io_mtx);
  if (out_done_count) *out_done_count = 0;

  int32_t rej = rejectIfFaulted(c, "move_to_many");
  if (rej) return rej;

  if (count == 0) {
    setErr(c, "");
    return TMC5160_OK;
  }

  // tol は正方向で扱う
  if (tol_ustep < 0) tol_ustep = -tol_ustep;

  for (uint32_t i = 0; i < count; ++i) {
    apply_limit_safety_always(h);
    if (c->fault_latched) return rejectIfFaulted(c, "move_to_many");

    int32_t tgt = positions[i];

    // RELATIVE 指定なら、その時点の XACTUAL からの差分
    if (flags & TMC_SEQ_RELATIVE) {
      int32_t cur = 0;
      int32_t rc = tmc5160_get_pos(h, &cur);
      if (rc != 0) return rc;
      tgt = cur + tgt;
    }

    // 移動開始（ここで limit hit していれば move_to 側で rejectIfFaulted が返る）
    int32_t rc = tmc5160_move_to(h, tgt);
    if (rc != 0) return rc;

    // WAIT 指定なら、到達 + vzero まで待つ
    if (flags & TMC_SEQ_WAIT_XTOL) {
      rc = wait_pos_reached_tol_with_safety(h, tgt, tol_ustep, timeout_ms_each);
      if (rc != 0) {
        if (c->fault_latched) return rejectIfFaulted(c, "move_to_many");
        return stop_on_error(c, rc, "move_to_many", "move_to_many: wait_pos_reached failed");
      }

      rc = wait_vzero_with_safety(h, timeout_ms_each);
      if (rc != 0) {
        if (c->fault_latched) return rejectIfFaulted(c, "move_to_many");
        return stop_on_error(c, rc, "move_to_many", "move_to_many: wait_vzero failed");
      }

      // dwell（停止中に待つ）
      dwell_with_safety(h, dwell_ms_each);
      if (c->fault_latched) return rejectIfFaulted(c, "move_to_many");
    } else {
      // 非WAITでも最低限のyield＋安全監視
      tiny_yield();
      apply_limit_safety_always(h);
      if (c->fault_latched) return rejectIfFaulted(c, "move_to_many");
      // dwell が必要ならここで dwell_with_safety(h, dwell_ms_each); してもOK
    }

    if (out_done_count) *out_done_count = (uint32_t)(i + 1);
  }

  setErr(c, "");
  apply_limit_safety_always(h);
  return TMC5160_OK;
}
