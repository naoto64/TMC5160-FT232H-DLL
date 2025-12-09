#pragma once
#include <cstdint>

namespace tmc5160 {

// General
static constexpr uint8_t REG_GCONF  = 0x00;
static constexpr uint8_t REG_GSTAT  = 0x01;
static constexpr uint8_t REG_IOIN   = 0x04;

// Ramp generator (0x20..)
static constexpr uint8_t REG_RAMPMODE  = 0x20;
static constexpr uint8_t REG_XACTUAL   = 0x21;
static constexpr uint8_t REG_VACTUAL   = 0x22;
static constexpr uint8_t REG_VSTART    = 0x23;
static constexpr uint8_t REG_A1        = 0x24;
static constexpr uint8_t REG_V1        = 0x25;
static constexpr uint8_t REG_AMAX      = 0x26;
static constexpr uint8_t REG_VMAX      = 0x27;
static constexpr uint8_t REG_DMAX      = 0x28;
static constexpr uint8_t REG_D1        = 0x2A;
static constexpr uint8_t REG_VSTOP     = 0x2B;
static constexpr uint8_t REG_TZEROWAIT = 0x2C;
static constexpr uint8_t REG_XTARGET   = 0x2D;

// Switch / latch
static constexpr uint8_t REG_SW_MODE   = 0x34;
static constexpr uint8_t REG_RAMP_STAT = 0x35;
static constexpr uint8_t REG_XLATCH    = 0x36;

// Encoder
static constexpr uint8_t REG_ENCMODE       = 0x38;
static constexpr uint8_t REG_X_ENC         = 0x39;
static constexpr uint8_t REG_ENC_CONST     = 0x3A;
static constexpr uint8_t REG_ENC_STATUS    = 0x3B;
static constexpr uint8_t REG_ENC_LATCH     = 0x3C;
static constexpr uint8_t REG_ENC_DEVIATION = 0x3D;

// RAMPMODE values
static constexpr uint32_t RAMPMODE_POSITION = 0;
static constexpr uint32_t RAMPMODE_VEL_POS  = 1;
static constexpr uint32_t RAMPMODE_VEL_NEG  = 2;
static constexpr uint32_t RAMPMODE_HOLD     = 3;

} // namespace tmc5160
