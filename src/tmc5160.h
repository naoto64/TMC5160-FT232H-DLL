#pragma once
#include <cstdint>
#include <string>
#include "ftdi_mpsse_spi.h"

class Tmc5160 {
public:
  explicit Tmc5160() = default;

  int openIndex(uint32_t index, uint32_t spi_hz, std::string& err) { return m_spi.openByIndex(index, spi_hz, err); }
  int openSerial(const char* serial, uint32_t spi_hz, std::string& err) { return m_spi.openBySerial(serial, spi_hz, err); }
  void close() { m_spi.close(); }

  int writeReg(uint8_t addr, uint32_t value, uint8_t* out_status, std::string& err);
  int readReg (uint8_t addr, uint32_t* out_value, uint8_t* out_status, std::string& err);

private:
  static uint32_t be_u32(const uint8_t* p);
  static void u32_to_be(uint32_t v, uint8_t* p);

  FtdiMpsseSpi m_spi;
};
