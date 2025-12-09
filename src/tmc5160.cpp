#include "tmc5160.h"

uint32_t Tmc5160::be_u32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
void Tmc5160::u32_to_be(uint32_t v, uint8_t* p) {
  p[0] = uint8_t((v >> 24) & 0xFF);
  p[1] = uint8_t((v >> 16) & 0xFF);
  p[2] = uint8_t((v >>  8) & 0xFF);
  p[3] = uint8_t((v >>  0) & 0xFF);
}

int Tmc5160::writeReg(uint8_t addr, uint32_t value, uint8_t* out_status, std::string& err) {
  uint8_t tx[5];
  uint8_t rx[5] = {0};

  tx[0] = uint8_t(addr | 0x80);
  u32_to_be(value, &tx[1]);

  int r = m_spi.xfer5(tx, rx, err);
  if (r) return r;

  if (out_status) *out_status = rx[0];
  return 0;
}

int Tmc5160::readReg(uint8_t addr, uint32_t* out_value, uint8_t* out_status, std::string& err) {
  if (!out_value) { err = "out_value null"; return -1; }

  uint8_t tx1[5] = { addr, 0,0,0,0 };
  uint8_t rx1[5] = {0};
  uint8_t tx2[5] = { addr, 0,0,0,0 };
  uint8_t rx2[5] = {0};

  // 1st: request (response is previous)
  int r = m_spi.xfer5(tx1, rx1, err);
  if (r) return r;

  // 2nd: receive data of request
  r = m_spi.xfer5(tx2, rx2, err);
  if (r) return r;

  if (out_status) *out_status = rx2[0];
  *out_value = be_u32(&rx2[1]);
  return 0;
}
