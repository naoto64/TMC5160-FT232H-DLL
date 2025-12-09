#pragma once

#include "ftd2xx.h"
#include <cstdint>
#include <string>
#include <mutex>

#ifdef _WIN32
  #include <windows.h>
#else
  typedef void* HANDLE;
#endif

class FtdiMpsseSpi {
public:
  FtdiMpsseSpi();
  ~FtdiMpsseSpi();

  int  openByIndex(uint32_t index, uint32_t spi_hz, std::string& err);
  int  openBySerial(const char* serial, uint32_t spi_hz, std::string& err);
  void close();

  int  xfer5(const uint8_t tx[5], uint8_t rx[5], std::string& err);

  bool isOpen() const { return m_open; }

private:
  int  openCommon(uint32_t spi_hz, std::string& err);

  int  writeAll(const uint8_t* p, uint32_t n, std::string& err);
  int  readExact(uint8_t* p, uint32_t n, uint32_t timeout_ms, std::string& err);

  int  setGpioLow(uint8_t value, uint8_t dir, std::string& err);
  int  mpsseSync(std::string& err);
  int  setSpiClock(uint32_t spi_hz, std::string& err);

  // RXに残っているゴミを捨てる
  int  drainRx(uint32_t max_bytes, uint32_t max_ms, std::string& err);
  // 互換：あなたのcppが drainRx(err) 1引数で呼んでても通るようにする
  int  drainRx(std::string& err) { return drainRx(1u << 20, 30, err); } // 1MB/30ms

private:
  FT_HANDLE   m_h;
  bool        m_open;
  std::mutex  m_mtx;

#ifdef _WIN32
  HANDLE      m_rx_event;   // FT_EVENT_RXCHAR 用
#endif
};
