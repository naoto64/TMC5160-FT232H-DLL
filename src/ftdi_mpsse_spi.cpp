#include "ftdi_mpsse_spi.h"

#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cstring>

#ifdef _WIN32
  #include <windows.h>
#endif

FtdiMpsseSpi::FtdiMpsseSpi() : m_h(nullptr), m_open(false)
{
#ifdef _WIN32
  m_rx_event = CreateEventA(nullptr, FALSE, FALSE, nullptr); // auto-reset
#endif
}

FtdiMpsseSpi::~FtdiMpsseSpi()
{
  close();
#ifdef _WIN32
  if (m_rx_event) {
    CloseHandle(m_rx_event);
    m_rx_event = nullptr;
  }
#endif
}

int FtdiMpsseSpi::writeAll(const uint8_t* p, uint32_t n, std::string& err)
{
  DWORD written = 0;
  FT_STATUS st = FT_Write(m_h, (LPVOID)p, (DWORD)n, &written);
  if (st != FT_OK || written != n) { err = "FT_Write failed"; return -3; }
  return 0;
}

int FtdiMpsseSpi::drainRx(uint32_t max_bytes, uint32_t max_ms, std::string& err)
{
  uint32_t drained = 0;
  auto t0 = std::chrono::steady_clock::now();

  for (;;) {
    DWORD q = 0;
    FT_GetQueueStatus(m_h, &q);
    if (q == 0) return 0;

    DWORD want = (DWORD)std::min<uint32_t>((uint32_t)q, std::min<uint32_t>(4096, max_bytes - drained));
    if (want == 0) return 0;

    uint8_t tmp[4096];
    DWORD r = 0;
    FT_STATUS st = FT_Read(m_h, tmp, want, &r);
    if (st != FT_OK) { err = "FT_Read(drain) failed"; return -3; }
    drained += (uint32_t)r;

    auto now = std::chrono::steady_clock::now();
    auto ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= max_ms) return 0;
    if (drained >= max_bytes) return 0;
  }
}

int FtdiMpsseSpi::readExact(uint8_t* p, uint32_t n, uint32_t timeout_ms, std::string& err)
{
  DWORD got = 0;
  auto t0 = std::chrono::steady_clock::now();

  while (got < n) {
    DWORD q = 0;
    FT_GetQueueStatus(m_h, &q);

    if (q > 0) {
      DWORD want = (DWORD)std::min<uint32_t>(n - got, (uint32_t)q);
      DWORD r = 0;
      FT_STATUS st = FT_Read(m_h, p + got, want, &r);
      if (st != FT_OK) { err = "FT_Read failed"; return -3; }
      got += r;
      continue;
    }

    // ここがジッタ源になりやすいので、WindowsならRXイベントを待つ
#ifdef _WIN32
    if (m_rx_event) {
      auto now = std::chrono::steady_clock::now();
      uint32_t ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
      if (ms >= timeout_ms) { err = "timeout waiting RX bytes"; return -4; }
      DWORD remain = timeout_ms - ms;
      DWORD slice  = (remain > 5) ? 5 : remain; // 5ms刻みで様子見
      WaitForSingleObject(m_rx_event, slice);
      continue;
    }
#endif

    // フォールバック（イベントが無い場合）
    auto now = std::chrono::steady_clock::now();
    auto ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
    if (ms >= timeout_ms) { err = "timeout waiting RX bytes"; return -4; }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return 0;
}

int FtdiMpsseSpi::setGpioLow(uint8_t value, uint8_t dir, std::string& err)
{
  uint8_t cmd[3] = { 0x80, value, dir };
  return writeAll(cmd, 3, err);
}

int FtdiMpsseSpi::mpsseSync(std::string& err)
{
  uint8_t b = 0xAA;
  int r = writeAll(&b, 1, err);
  if (r) return r;

  uint8_t rx[2] = {0};
  r = readExact(rx, 2, 200, err);
  if (r) return r;

  if (!(rx[0] == 0xFA && rx[1] == 0xAA)) {
    err = "MPSSE sync failed (expected 0xFA 0xAA)";
    return -3;
  }
  return 0;
}

int FtdiMpsseSpi::setSpiClock(uint32_t spi_hz, std::string& err)
{
  if (spi_hz == 0) { err = "spi_hz=0"; return -1; }

  double div_d = (60e6 / (2.0 * (double)spi_hz)) - 1.0;
  if (div_d < 0) div_d = 0;
  if (div_d > 65535) div_d = 65535;
  uint16_t div = (uint16_t)(div_d + 0.5);

  uint8_t cmd[6] = {
    0x8A, // disable div/5
    0x97, // disable adaptive
    0x8D, // disable 3-phase
    0x86, // set divisor
    (uint8_t)(div & 0xFF),
    (uint8_t)((div >> 8) & 0xFF)
  };
  return writeAll(cmd, 6, err);
}

int FtdiMpsseSpi::openCommon(uint32_t spi_hz, std::string& err)
{
  FT_ResetDevice(m_h);
  FT_SetUSBParameters(m_h, 65536, 65535);
  FT_SetChars(m_h, false, 0, false, 0);
  FT_SetTimeouts(m_h, 1000, 1000);
  FT_SetLatencyTimer(m_h, 1);

  // enable MPSSE
  FT_SetBitMode(m_h, 0x0, 0x00);
  FT_SetBitMode(m_h, 0x0, 0x02);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // purge junk
  FT_Purge(m_h, FT_PURGE_RX | FT_PURGE_TX);

#ifdef _WIN32
  // RXイベント通知（readExactのWait用）
  if (m_rx_event) {
    FT_SetEventNotification(m_h, FT_EVENT_RXCHAR, m_rx_event);
  }
#endif

  // sync（これをちゃんと読まないと後で 0xFA が混ざる）
  int r = mpsseSync(err);
  if (r) return r;

  // 念のため、sync後のゴミを捨てる
  (void)drainRx(err);

  // clock
  r = setSpiClock(spi_hz, err);
  if (r) return r;

  // GPIO (ADBUS): bit0=SK(out), bit1=DO(out), bit2=DI(in), bit3=CS(out)
  // dir: 1111_1011 = 0xFB
  // val: CS high = 0x08, SK low
  r = setGpioLow(0x08, 0xFB, err);
  if (r) return r;

  return 0;
}

int FtdiMpsseSpi::openByIndex(uint32_t index, uint32_t spi_hz, std::string& err)
{
  std::lock_guard<std::mutex> lk(m_mtx);
  if (m_open) { err = "already open"; return -1; }

  FT_STATUS st = FT_Open((int)index, &m_h);
  if (st != FT_OK || !m_h) { err = "FT_Open failed"; m_h = nullptr; return -3; }

  int r = openCommon(spi_hz, err);
  if (r) { FT_Close(m_h); m_h=nullptr; return r; }

  m_open = true;
  return 0;
}

int FtdiMpsseSpi::openBySerial(const char* serial, uint32_t spi_hz, std::string& err)
{
  std::lock_guard<std::mutex> lk(m_mtx);
  if (m_open) { err = "already open"; return -1; }
  if (!serial || !serial[0]) { err = "serial empty"; return -1; }

  FT_STATUS st = FT_OpenEx((PVOID)serial, FT_OPEN_BY_SERIAL_NUMBER, &m_h);
  if (st != FT_OK || !m_h) { err = "FT_OpenEx(serial) failed"; m_h=nullptr; return -3; }

  int r = openCommon(spi_hz, err);
  if (r) { FT_Close(m_h); m_h=nullptr; return r; }

  m_open = true;
  return 0;
}

void FtdiMpsseSpi::close()
{
  std::lock_guard<std::mutex> lk(m_mtx);
  if (!m_open) return;
  FT_Close(m_h);
  m_h = nullptr;
  m_open = false;
}

int FtdiMpsseSpi::xfer5(const uint8_t tx[5], uint8_t rx[5], std::string& err)
{
  std::lock_guard<std::mutex> lk(m_mtx);
  if (!m_open) { err = "not open"; return -2; }

  // FT_Purge を毎回やるとばらつきが出やすいので、RXを読むだけで「整える」
  (void)drainRx(err);

  uint8_t head[3] = { 0x80, 0x00, 0xFB }; // CS low
  uint8_t cmd[3]  = { 0x31, 0x04, 0x00 }; // clock bytes in/out, MSB, length=5
  uint8_t tail[3] = { 0x80, 0x08, 0xFB }; // CS high

  std::vector<uint8_t> buf;
  buf.reserve(3 + 3 + 5 + 3);
  buf.insert(buf.end(), head, head+3);
  buf.insert(buf.end(), cmd,  cmd+3);
  buf.insert(buf.end(), tx,   tx+5);
  buf.insert(buf.end(), tail, tail+3);

  int r = writeAll(buf.data(), (uint32_t)buf.size(), err);
  if (r) return r;

  r = readExact(rx, 5, 200, err);
  if (r) return r;

  return 0;
}
