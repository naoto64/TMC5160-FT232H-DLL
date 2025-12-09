# example_mm_current_dll.py  (tmc5160.dll と同じフォルダで実行)
# - 今のDLL(tmc5160_dll.h)に合わせた ctypes 定義
# - mm / mm/s / mm/s^2 でランプ設定
# - move_to_many / home / get_ref_pins / dump_inputs 対応

import ctypes as C
import time
import os
from ctypes import c_uint32, c_uint8, c_int32, c_void_p, byref

dll_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tmc5160.dll")

# =========================
# 機構パラメータ（ここだけ合わせればOK）
# =========================
STEPS_PER_REV = 200      # モータのフルステップ/回転
MICROSTEPS    = 256      # マイクロステップ（DLL側と合わせる）
MM_PER_REV    = 4.0      # 1回転で何mm進むか（ボールねじピッチ等）
ORIGIN_MM     = 0.0

# TMC5160 の内部クロック fCLK [Hz]
FCLK_HZ       = 12_000_000

# 到達判定（Noneなら「1µstep相当」を自動採用）
POS_EPS_MM    = None

# =========================
# mm <-> ustep 変換
# =========================
def usteps_per_mm() -> float:
    return (STEPS_PER_REV * MICROSTEPS) / float(MM_PER_REV)

def default_eps_mm() -> float:
    return 1.0 / usteps_per_mm()

def mm_to_ustep(mm: float) -> int:
    return int(round((mm - ORIGIN_MM) * usteps_per_mm()))

def ustep_to_mm(x: int) -> float:
    return ORIGIN_MM + (float(x) / usteps_per_mm())


# =========================
# 物理単位(mm/s, mm/s^2) → TMC5160 レジスタ値 変換
# =========================
def _clamp_u32(x: int) -> int:
    if x < 0: return 0
    if x > 0xFFFFFFFF: return 0xFFFFFFFF
    return int(x)

def mm_s_to_v5160(v_mm_s: float, fclk_hz: int = FCLK_HZ) -> int:
    # v_reg = v_ustep_s * 2^24 / fCLK
    v_ustep_s = v_mm_s * usteps_per_mm()
    v_reg = int(round(v_ustep_s * (1 << 24) / float(fclk_hz)))
    return _clamp_u32(v_reg)

def mm_s2_to_a5160(a_mm_s2: float, fclk_hz: int = FCLK_HZ) -> int:
    # a_reg = a_ustep_s2 * (512*256) * 2^26 / fCLK^2
    a_ustep_s2 = a_mm_s2 * usteps_per_mm()
    a_reg = int(round(a_ustep_s2 * (512 * 256) * (1 << 26) / float(fclk_hz * fclk_hz)))
    return _clamp_u32(a_reg)

# 逆変換（デバッグ用）
def v5160_to_mm_s(v_reg: int, fclk_hz: int = FCLK_HZ) -> float:
    v_ustep_s = float(v_reg) * float(fclk_hz) / float(1 << 24)
    return v_ustep_s / usteps_per_mm()

def a5160_to_mm_s2(a_reg: int, fclk_hz: int = FCLK_HZ) -> float:
    a_ustep_s2 = float(a_reg) * (float(fclk_hz) * float(fclk_hz)) / float((512 * 256) * (1 << 26))
    return a_ustep_s2 / usteps_per_mm()


# =========================
# DLL load & prototypes (今のDLL用)
# =========================
dll = C.CDLL(dll_path)
H = c_void_p

# --- constants from header ---
TMC_SEQ_ABSOLUTE  = 0
TMC_SEQ_RELATIVE  = 1 << 0
TMC_SEQ_WAIT_XTOL = 1 << 1

TMC_HOME_ACTIVE_LOW = 1 << 0
TMC_HOME_USE_LATCH  = 1 << 1
TMC_HOME_SOFTSTOP   = 1 << 2

# --- core ---
dll.tmc5160_open_index.argtypes = [c_uint32, c_uint32, C.POINTER(H)]
dll.tmc5160_open_index.restype  = c_int32
dll.tmc5160_close.argtypes      = [H]
dll.tmc5160_close.restype       = None

dll.tmc5160_setup_motor.argtypes = [H, c_uint8, c_uint8, c_uint8, c_int32, c_uint32]
dll.tmc5160_setup_motor.restype  = c_int32

dll.tmc5160_set_ramp_params.argtypes = [H, c_uint32,c_uint32,c_uint32,c_uint32,c_uint32,c_uint32,c_uint32,c_uint32]
dll.tmc5160_set_ramp_params.restype  = c_int32

dll.tmc5160_set_pos.argtypes = [H, c_int32]
dll.tmc5160_set_pos.restype  = c_int32
dll.tmc5160_get_pos.argtypes = [H, C.POINTER(c_int32)]
dll.tmc5160_get_pos.restype  = c_int32
dll.tmc5160_move_to.argtypes = [H, c_int32]
dll.tmc5160_move_to.restype  = c_int32

dll.tmc5160_last_error_global.argtypes = [C.c_char_p, c_uint32]
dll.tmc5160_last_error_global.restype  = c_int32

# --- dump_inputs (文字列返し版) ---
HAS_DUMP = hasattr(dll, "tmc5160_dump_inputs")
if HAS_DUMP:
    dll.tmc5160_dump_inputs.argtypes = [H,
    C.POINTER(c_uint32), C.POINTER(c_uint32), C.POINTER(c_uint32)]
    dll.tmc5160_dump_inputs.restype = c_int32

# optional APIs
HAS_MOVE_TO_MANY = hasattr(dll, "tmc5160_move_to_many")
if HAS_MOVE_TO_MANY:
    dll.tmc5160_move_to_many.argtypes = [
        H, C.POINTER(c_int32), c_uint32,
        c_uint32, c_uint32, c_uint32, c_int32,
        C.POINTER(c_uint32)
    ]
    dll.tmc5160_move_to_many.restype = c_int32

HAS_HOME = hasattr(dll, "tmc5160_home")
if HAS_HOME:
    # int32_t tmc5160_home(H, dir, flags, v_fast, v_slow, backoff_ustep, timeout_ms_each, home_pos_ustep, out_latched)
    dll.tmc5160_home.argtypes = [H, c_int32, c_uint32, c_int32, c_int32, c_int32, c_uint32, c_int32, C.POINTER(c_int32)]
    dll.tmc5160_home.restype  = c_int32

HAS_REF_PINS = hasattr(dll, "tmc5160_get_ref_pins")
if HAS_REF_PINS:
    dll.tmc5160_get_ref_pins.argtypes = [H, C.POINTER(c_int32), C.POINTER(c_int32)]
    dll.tmc5160_get_ref_pins.restype  = c_int32

# --- move_to_limit ---
HAS_MOVE_TO_LIMIT = hasattr(dll, "tmc5160_move_to_limit")
if HAS_MOVE_TO_LIMIT:
    # int32_t tmc5160_move_to_limit(H, dir, velocity, timeout_ms, out_hit_dir)
    dll.tmc5160_move_to_limit.argtypes = [H, c_int32, c_int32, c_uint32, C.POINTER(c_int32)]
    dll.tmc5160_move_to_limit.restype  = c_int32

def last_err_global() -> str:
    buf = C.create_string_buffer(1024)
    dll.tmc5160_last_error_global(buf, 1024)
    return buf.value.decode(errors="ignore")

def dump_inputs(h: H, tag: str = ""):
    ioin = c_uint32(0)
    rs   = c_uint32(0)
    sw   = c_uint32(0)
    rc = dll.tmc5160_dump_inputs(h, byref(ioin), byref(rs), byref(sw))

    REFL = (ioin.value >> 0) & 1
    REFR = (ioin.value >> 1) & 1

    stopL  = (rs.value >> 0) & 1
    stopR  = (rs.value >> 1) & 1
    latchL = (rs.value >> 2) & 1
    latchR = (rs.value >> 3) & 1
    evL    = (rs.value >> 4) & 1
    evR    = (rs.value >> 5) & 1
    vzero  = (rs.value >>10) & 1
    posr   = (rs.value >> 9) & 1  # 参考

    print(f"[dump_inputs] {tag}: rc={rc}  "
          f"IOIN=0x{ioin.value:08X} REFL={REFL} REFR={REFR}  "
          f"RAMPSTAT=0x{rs.value:08X} stopL={stopL} stopR={stopR} "
          f"latchL={latchL} latchR={latchR} evL={evL} evR={evR} vzero={vzero} posR={posr}  "
          f"SW_MODE=0x{sw.value:08X}")

    return rc, ioin.value, rs.value, sw.value

def check(rc, msg="", h: H = None):
    if rc != 0:
        # 失敗時は必ず状態も出す
        if h is not None:
            dump_inputs(h, f"on_error({msg})")
        raise RuntimeError(f"{msg} rc={rc} err='{last_err_global()}'")


# =========================
# 高レベル：mm指定で動かす
# =========================
def get_pos_mm(h: H) -> float:
    pos = c_int32()
    check(dll.tmc5160_get_pos(h, byref(pos)), "get_pos", h)
    return ustep_to_mm(pos.value)

def wait_reach_mm(h: H, target_mm: float, timeout_s: float = 10.0, eps_mm: float = None):
    if eps_mm is None:
        eps_mm = default_eps_mm() if (POS_EPS_MM is None) else max(POS_EPS_MM, default_eps_mm())

    t0 = time.perf_counter()
    while True:
        cur_mm = get_pos_mm(h)
        if abs(cur_mm - target_mm) <= eps_mm:
            return
        if (time.perf_counter() - t0) >= timeout_s:
            dump_inputs(h, "wait_reach_mm timeout")
            raise TimeoutError(f"timeout: target={target_mm}mm current={cur_mm}mm eps={eps_mm}mm")

def move_to_mm(h: H, mm: float, timeout_s: float = 10.0, eps_mm: float = None):
    xt = mm_to_ustep(mm)
    print(f"move_to_mm: {mm:.3f} mm -> XTARGET={xt}")
    check(dll.tmc5160_move_to(h, xt), "move_to", h)
    wait_reach_mm(h, mm, timeout_s=timeout_s, eps_mm=eps_mm)


def set_ramp_params_mm(h: H,
                       vstart_mm_s: float,
                       a1_mm_s2: float,
                       v1_mm_s: float,
                       amax_mm_s2: float,
                       vmax_mm_s: float,
                       dmax_mm_s2: float,
                       d1_mm_s2: float,
                       vstop_mm_s: float,
                       fclk_hz: int = FCLK_HZ):
    vstart = mm_s_to_v5160(vstart_mm_s, fclk_hz)
    v1     = mm_s_to_v5160(v1_mm_s,     fclk_hz)
    vmax   = mm_s_to_v5160(vmax_mm_s,   fclk_hz)
    vstop  = mm_s_to_v5160(vstop_mm_s,  fclk_hz)

    a1     = mm_s2_to_a5160(a1_mm_s2,   fclk_hz)
    amax   = mm_s2_to_a5160(amax_mm_s2, fclk_hz)
    dmax   = mm_s2_to_a5160(dmax_mm_s2, fclk_hz)
    d1     = mm_s2_to_a5160(d1_mm_s2,   fclk_hz)

    print("[set_ramp_params_mm] mm/s, mm/s^2 => reg")
    print(f"  VSTART={vstart} ({vstart_mm_s} mm/s)")
    print(f"  V1    ={v1} ({v1_mm_s} mm/s)")
    print(f"  VMAX  ={vmax} ({vmax_mm_s} mm/s)")
    print(f"  VSTOP ={vstop} ({vstop_mm_s} mm/s)")
    print(f"  A1    ={a1} ({a1_mm_s2} mm/s^2)")
    print(f"  AMAX  ={amax} ({amax_mm_s2} mm/s^2)")
    print(f"  DMAX  ={dmax} ({dmax_mm_s2} mm/s^2)")
    print(f"  D1    ={d1} ({d1_mm_s2} mm/s^2)")
    print(f"  debug(back-calc) VMAX={v5160_to_mm_s(vmax,fclk_hz):.4f} mm/s  AMAX={a5160_to_mm_s2(amax,fclk_hz):.4f} mm/s^2")

    check(dll.tmc5160_set_ramp_params(h, vstart, a1, v1, amax, vmax, dmax, d1, vstop), "set_ramp_params", h)


def move_many_mm(h: H, mm_list, dwell_ms_each=0, timeout_s_each=15.0,
                 use_abs=True, use_xtol=True, tol_mm=None):
    if tol_mm is None:
        tol_mm = default_eps_mm() if (POS_EPS_MM is None) else max(POS_EPS_MM, default_eps_mm())

    if HAS_MOVE_TO_MANY:
        pts = (c_int32 * len(mm_list))()
        for i, mm in enumerate(mm_list):
            pts[i] = mm_to_ustep(mm) if use_abs else int(round(mm * usteps_per_mm()))

        done = c_uint32(0)
        flags = (TMC_SEQ_ABSOLUTE if use_abs else TMC_SEQ_RELATIVE) | (TMC_SEQ_WAIT_XTOL if use_xtol else 0)

        tol_ustep = int(round(tol_mm * usteps_per_mm()))
        timeout_ms_each = int(timeout_s_each * 1000)

        print(f"[move_many_mm] DLL bulk: count={len(mm_list)} dwell={dwell_ms_each}ms "
              f"timeout_each={timeout_ms_each}ms flags={flags} tol_ustep={tol_ustep} (≈{tol_mm} mm)")

        t0 = time.perf_counter()
        check(dll.tmc5160_move_to_many(h, pts, len(mm_list),
                                       int(dwell_ms_each),
                                       int(timeout_ms_each),
                                       int(flags),
                                       int(tol_ustep),
                                       byref(done)), "move_to_many", h)
        t1 = time.perf_counter()
        print(f"{(t1-t0)*1000:.2f} ms  done_count={done.value}")
        return

    # fallback (DLLに無い場合)
    if use_abs:
        for mm in mm_list:
            move_to_mm(h, mm, timeout_s=timeout_s_each, eps_mm=tol_mm)
            if dwell_ms_each:
                time.sleep(dwell_ms_each / 1000.0)
    else:
        cur = get_pos_mm(h)
        for dmm in mm_list:
            cur += dmm
            move_to_mm(h, cur, timeout_s=timeout_s_each, eps_mm=tol_mm)
            if dwell_ms_each:
                time.sleep(dwell_ms_each / 1000.0)


def read_ref_pins(h: H):
    """REFL/REFRの生状態(High/Low)を読む。基板配線チェック用。"""
    if not HAS_REF_PINS:
        print("[ref_pins] API not found in DLL")
        return None
    refl = c_int32()
    refr = c_int32()
    check(dll.tmc5160_get_ref_pins(h, byref(refl), byref(refr)), "get_ref_pins", h)
    return (refl.value, refr.value)


def home_mm(h: H,
            dir_left: bool = True,
            active_low: bool = True,
            use_latch: bool = True,
            softstop: bool = False,
            v_fast_mm_s: float = 2.0,
            v_slow_mm_s: float = 0.3,
            backoff_mm: float = 1.0,
            home_pos_mm: float = 0.0,
            timeout_s_each: float = 15.0):
    """
    今のDLLの tmc5160_home を mm 指定で呼ぶラッパ
    - dir_left=True で左(-1), Falseで右(+1)
    - v_fast/v_slow は rotate用の速度レジスタ値が必要なので mm/s→vreg で変換
    """
    if not HAS_HOME:
        raise RuntimeError("tmc5160_home not found in DLL")

    flags = 0
    if active_low: flags |= TMC_HOME_ACTIVE_LOW
    if use_latch:  flags |= TMC_HOME_USE_LATCH
    if softstop:   flags |= TMC_HOME_SOFTSTOP

    dir_ = -1 if dir_left else +1

    v_fast = mm_s_to_v5160(v_fast_mm_s)
    v_slow = mm_s_to_v5160(v_slow_mm_s)
    backoff_ustep  = int(round(abs(backoff_mm) * usteps_per_mm()))
    home_pos_ustep = mm_to_ustep(home_pos_mm)
    timeout_ms_each = int(timeout_s_each * 1000)

    print(f"[home_mm] dir={dir_} flags={flags} v_fast(reg)={v_fast} v_slow(reg)={v_slow} "
          f"backoff={backoff_ustep}ustep home_pos={home_pos_ustep}ustep timeout={timeout_ms_each}ms")

    dump_inputs(h, "before home")
    lat = c_int32()
    check(dll.tmc5160_home(h, dir_, flags, int(v_fast), int(v_slow),
                           int(backoff_ustep), int(timeout_ms_each),
                           int(home_pos_ustep), byref(lat)), "home", h)
    dump_inputs(h, "after home")

    print(f"[home_mm] latched={lat.value} ustep  (≈{ustep_to_mm(lat.value):.6f} mm)")
    return lat.value

def move_to_limit_mm(h: H,
                     dir_left: bool = True,
                     v_mm_s: float = 5.0,
                     timeout_s: float = 15.0):
    """
    DLLの tmc5160_move_to_limit を mm/s 指定で呼ぶラッパ
    - dir_left=True で LEFT(-1), Falseで RIGHT(+1)
    - v_mm_s は mm/s → vreg 変換して渡す（DLLは rotate と同じ velocity 単位）
    - リミット到達は rc=0(TMC5160_OK) で返る（エラー扱いにしない仕様）
    """
    if not HAS_MOVE_TO_LIMIT:
        raise RuntimeError("tmc5160_move_to_limit not found in DLL")

    dir_ = -1 if dir_left else +1
    vreg = mm_s_to_v5160(abs(v_mm_s))
    timeout_ms = int(timeout_s * 1000)

    hit = c_int32(0)
    dump_inputs(h, f"before move_to_limit ({'L' if dir_left else 'R'})")
    rc = dll.tmc5160_move_to_limit(h, int(dir_), int(vreg), int(timeout_ms), byref(hit))
    check(rc, "move_to_limit", h)
    dump_inputs(h, f"after move_to_limit hit={hit.value}")

    # hit.value: -1(L), +1(R), 0(両方/不明)
    return hit.value

# =========================
# main
# =========================
def main():
    h = H()
    check(dll.tmc5160_open_index(0, 1_000_000, byref(h)), "open_index")  # SPI 1MHz

    try:
        # モータ初期化
        check(dll.tmc5160_setup_motor(h, 3, 5, 6, MICROSTEPS, 0), "setup_motor", h)

        # ランプ(mm/s, mm/s^2)で設定
        VSTART_MM_S = 0.1
        VSTOP_MM_S  = 0.1
        VMAX_MM_S   = 5.0
        AMAX_MM_S2  = 400.0

        set_ramp_params_mm(
            h,
            vstart_mm_s=VSTART_MM_S,
            a1_mm_s2=AMAX_MM_S2,
            v1_mm_s=VMAX_MM_S,
            amax_mm_s2=AMAX_MM_S2,
            vmax_mm_s=VMAX_MM_S,
            dmax_mm_s2=AMAX_MM_S2,
            d1_mm_s2=AMAX_MM_S2,
            vstop_mm_s=VSTOP_MM_S
        )

        # 現在位置を原点扱いにしたい場合
        check(dll.tmc5160_set_pos(h, mm_to_ustep(ORIGIN_MM)), "set_pos", h)

        # 参考：REFL/REFR生状態
        pins = read_ref_pins(h)
        if pins is not None:
            print(f"REFL={pins[0]} REFR={pins[1]}  (生状態)")

        # 例：RIGHTリミットまで移動（到達はOK返り）
        hit = move_to_limit_mm(h, dir_left=False, v_mm_s=10.0, timeout_s=20.0)
        print(f"[move_to_limit_mm] hit={hit}  (+1=R, -1=L, 0=unknown)")

        # 原点復帰（左へ）
        home_mm(
            h,
            dir_left=True,
            active_low=True,     # スイッチが押されたらLowならTrue
            use_latch=True,      # XLATCH使う
            softstop=False,
            v_fast_mm_s=5.0,
            v_slow_mm_s=1.0,
            backoff_mm=10.0,
            home_pos_mm=0.0,
            timeout_s_each=15.0
        )
        time.sleep(2)

        set_ramp_params_mm(
            h,
            vstart_mm_s=VSTART_MM_S,
            a1_mm_s2=AMAX_MM_S2,
            v1_mm_s=VMAX_MM_S,
            amax_mm_s2=AMAX_MM_S2,
            vmax_mm_s=VMAX_MM_S,
            dmax_mm_s2=AMAX_MM_S2,
            d1_mm_s2=AMAX_MM_S2,
            vstop_mm_s=VSTOP_MM_S
        )

        seq = [-3.0, 0.0] * 1  # mm 指定
        move_many_mm(
            h,
            mm_list=seq,
            dwell_ms_each=0,
            timeout_s_each=15.0,
            use_abs=True,
            use_xtol=True,
            tol_mm=0.01
        )
    finally:
        if h:
            dll.tmc5160_close(h)
            h = None

    dll.tmc5160_close(h)
    print("done")


if __name__ == "__main__":
    main()
