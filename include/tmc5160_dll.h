// include/tmc5160_dll.h
#pragma once
#include <stdint.h>

#ifdef _WIN32
  #if defined(tmc5160_EXPORTS) || defined(TMC5160_EXPORTS) || defined(TMC5160_DLL_BUILD)
    #define TMC_EXPORT __declspec(dllexport)
  #else
    #define TMC_EXPORT __declspec(dllimport)
  #endif
#else
  #define TMC_EXPORT __attribute__((visibility("default")))
#endif


#ifdef __cplusplus
  #define TMC_API extern "C" TMC_EXPORT
#else
  #define TMC_API TMC_EXPORT
#endif

typedef void* TMC5160_Handle;

// ---- return codes (your python log used -4 as timeout) ----
enum {
  TMC5160_OK              = 0,
  TMC5160_ERR_INVALID_ARG = -1,
  TMC5160_ERR_IO          = -2,
  TMC5160_ERR_NOT_SUPPORTED = -3,
  TMC5160_ERR_TIMEOUT     = -4,
};

// ---- sequence flags (move_to_many) ----
enum {
  TMC_SEQ_ABSOLUTE  = 0u,
  TMC_SEQ_RELATIVE  = 1u << 0,
  TMC_SEQ_WAIT_XTOL = 1u << 1,
};

// ---- homing flags ----
enum {
  TMC_HOME_ACTIVE_LOW  = 1u << 0,
  TMC_HOME_USE_LATCH   = 1u << 1,
  TMC_HOME_SOFTSTOP    = 1u << 2,
  TMC_HOME_ACTIVE_HIGH = 1u << 3, // 0x08
};

// ---- error ----
TMC_API int32_t tmc5160_last_error_global(char* out, uint32_t out_len);
TMC_API int32_t tmc5160_last_error(TMC5160_Handle h, char* out, uint32_t out_len);

// ---- enumerate (FTDI) ----
TMC_API int32_t tmc5160_get_device_count(uint32_t* out_count);
TMC_API int32_t tmc5160_get_device_info(uint32_t index,
                                        char* out_serial, uint32_t serial_len,
                                        char* out_desc,   uint32_t desc_len);

// ---- open/close ----
TMC_API int32_t tmc5160_open_index(uint32_t device_index, uint32_t spi_hz, TMC5160_Handle* out_h);
TMC_API int32_t tmc5160_open_serial(const char* serial, uint32_t spi_hz, TMC5160_Handle* out_h);
TMC_API void    tmc5160_close(TMC5160_Handle h);

// ---- low-level ----
TMC_API int32_t tmc5160_write_reg(TMC5160_Handle h, uint8_t addr, uint32_t value, uint8_t* out_status);
TMC_API int32_t tmc5160_read_reg (TMC5160_Handle h, uint8_t addr, uint32_t* out_value, uint8_t* out_status);

// ---- debug ----
TMC_API int32_t tmc5160_dump_inputs(TMC5160_Handle h,
                                   uint32_t* out_ioin,
                                   uint32_t* out_rampstat,
                                   uint32_t* out_sw_mode);

// ---- fault latch ----
TMC_API int32_t tmc5160_clear_fault(TMC5160_Handle h);

// ---- motor basic ----
TMC_API int32_t tmc5160_setup_motor(TMC5160_Handle h,
                                   uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                                   int32_t microsteps,
                                   uint32_t chopconf_base);

TMC_API int32_t tmc5160_set_ramp_params(TMC5160_Handle h,
                                       uint32_t vstart, uint32_t a1, uint32_t v1,
                                       uint32_t amax,   uint32_t vmax,
                                       uint32_t dmax,   uint32_t d1,
                                       uint32_t vstop);

TMC_API int32_t tmc5160_set_pos(TMC5160_Handle h, int32_t pos);
TMC_API int32_t tmc5160_get_pos(TMC5160_Handle h, int32_t* out_pos);
TMC_API int32_t tmc5160_get_vel(TMC5160_Handle h, int32_t* out_vel);

TMC_API int32_t tmc5160_move_to(TMC5160_Handle h, int32_t pos);
TMC_API int32_t tmc5160_move_by(TMC5160_Handle h, int32_t delta);
TMC_API int32_t tmc5160_rotate (TMC5160_Handle h, int32_t velocity);
TMC_API int32_t tmc5160_stop   (TMC5160_Handle h);

TMC_API int32_t tmc5160_is_pos_reached(TMC5160_Handle h, int* out_yes);
TMC_API int32_t tmc5160_wait_pos_reached(TMC5160_Handle h, uint32_t timeout_ms);

// ---- bulk sequence ----
TMC_API int32_t tmc5160_move_to_many(TMC5160_Handle h,
                                    const int32_t* positions,
                                    uint32_t count,
                                    uint32_t dwell_ms_each,
                                    uint32_t timeout_ms_each,
                                    uint32_t flags,
                                    int32_t tol_ustep,
                                    uint32_t* out_done_count);

// ---- encoder ----
TMC_API int32_t tmc5160_setup_encoder(TMC5160_Handle h, uint32_t encmode);
TMC_API int32_t tmc5160_get_enc_pos (TMC5160_Handle h, int32_t* out_pos);
TMC_API int32_t tmc5160_set_enc_pos (TMC5160_Handle h, int32_t pos);

// ---- ref pins ----
TMC_API int32_t tmc5160_get_ref_pins(TMC5160_Handle h, int* out_refl, int* out_refr);

// ---- homing ----
TMC_API int32_t tmc5160_home(TMC5160_Handle h,
                             int32_t dir,
                             uint32_t flags,
                             int32_t v_fast,
                             int32_t v_slow,
                             int32_t backoff_ustep,
                             uint32_t timeout_ms_each,
                             int32_t home_pos_ustep,
                             int32_t* out_latched_ustep);
                             
// Move until limit switch is pressed (LEFT/RIGHT).
// - dir: -1 = LEFT, +1 = RIGHT
// - velocity: magnitude (same unit as rotate). sign is ignored.
// - Returns TMC5160_OK when a limit is hit (no fault latch, no error).
// - out_hit_dir: -1 (LEFT hit), +1 (RIGHT hit), 0 (both/unknown). Can be NULL.
TMC_API int32_t tmc5160_move_to_limit(TMC5160_Handle h,
                                      int32_t dir,
                                      int32_t velocity,
                                      uint32_t timeout_ms,
                                      int32_t* out_hit_dir);
