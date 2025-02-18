#pragma once
#include "tmc22xx_registers.h"
#include "tmc22xx_api.h"
#include "tmc22xx_events.h"

namespace esphome {
namespace tmc22xx {

#define B1_TO_BINARY_PATTERN "%c"
#define B2_TO_BINARY_PATTERN B1_TO_BINARY_PATTERN B1_TO_BINARY_PATTERN
#define B3_TO_BINARY_PATTERN B2_TO_BINARY_PATTERN B1_TO_BINARY_PATTERN
#define B4_TO_BINARY_PATTERN B2_TO_BINARY_PATTERN B2_TO_BINARY_PATTERN
#define B8_TO_BINARY_PATTERN B4_TO_BINARY_PATTERN B4_TO_BINARY_PATTERN
#define B10_TO_BINARY_PATTERN B8_TO_BINARY_PATTERN B2_TO_BINARY_PATTERN
#define B12_TO_BINARY_PATTERN B8_TO_BINARY_PATTERN B4_TO_BINARY_PATTERN
#define B16_TO_BINARY_PATTERN B8_TO_BINARY_PATTERN B8_TO_BINARY_PATTERN
#define B20_TO_BINARY_PATTERN B16_TO_BINARY_PATTERN B4_TO_BINARY_PATTERN
#define B24_TO_BINARY_PATTERN B16_TO_BINARY_PATTERN B8_TO_BINARY_PATTERN

#define B1_TO_BINARY(bit) ((bit) ? '1' : '0')

#define B2_TO_BINARY(bits) (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2))

#define B3_TO_BINARY(bits) (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4))

#define B4_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8))

#define B8_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8)), \
      (B1_TO_BINARY((bits) &0x10)), (B1_TO_BINARY((bits) &0x20)), (B1_TO_BINARY((bits) &0x40)), \
      (B1_TO_BINARY((bits) &0x80))

#define B10_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8)), \
      (B1_TO_BINARY((bits) &0x10)), (B1_TO_BINARY((bits) &0x20)), (B1_TO_BINARY((bits) &0x40)), \
      (B1_TO_BINARY((bits) &0x80)), (B1_TO_BINARY((bits) &0x100)), (B1_TO_BINARY((bits) &0x200))

#define B12_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8)), \
      (B1_TO_BINARY((bits) &0x10)), (B1_TO_BINARY((bits) &0x20)), (B1_TO_BINARY((bits) &0x40)), \
      (B1_TO_BINARY((bits) &0x80)), (B1_TO_BINARY((bits) &0x100)), (B1_TO_BINARY((bits) &0x200)), \
      (B1_TO_BINARY((bits) &0x400)), (B1_TO_BINARY((bits) &0x800))

#define B16_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8)), \
      (B1_TO_BINARY((bits) &0x10)), (B1_TO_BINARY((bits) &0x20)), (B1_TO_BINARY((bits) &0x40)), \
      (B1_TO_BINARY((bits) &0x80)), (B1_TO_BINARY((bits) &0x100)), (B1_TO_BINARY((bits) &0x200)), \
      (B1_TO_BINARY((bits) &0x400)), (B1_TO_BINARY((bits) &0x800)), (B1_TO_BINARY((bits) &0x1000)), \
      (B1_TO_BINARY((bits) &0x2000)), (B1_TO_BINARY((bits) &0x4000)), (B1_TO_BINARY((bits) &0x8000))

#define B20_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8)), \
      (B1_TO_BINARY((bits) &0x10)), (B1_TO_BINARY((bits) &0x20)), (B1_TO_BINARY((bits) &0x40)), \
      (B1_TO_BINARY((bits) &0x80)), (B1_TO_BINARY((bits) &0x100)), (B1_TO_BINARY((bits) &0x200)), \
      (B1_TO_BINARY((bits) &0x400)), (B1_TO_BINARY((bits) &0x800)), (B1_TO_BINARY((bits) &0x1000)), \
      (B1_TO_BINARY((bits) &0x2000)), (B1_TO_BINARY((bits) &0x4000)), (B1_TO_BINARY((bits) &0x8000)), \
      (B1_TO_BINARY((bits) &0x10000)), (B1_TO_BINARY((bits) &0x20000)), (B1_TO_BINARY((bits) &0x40000)), \
      (B1_TO_BINARY((bits) &0x80000))

#define B24_TO_BINARY(bits) \
  (B1_TO_BINARY((bits) &0x1)), (B1_TO_BINARY((bits) &0x2)), (B1_TO_BINARY((bits) &0x4)), (B1_TO_BINARY((bits) &0x8)), \
      (B1_TO_BINARY((bits) &0x10)), (B1_TO_BINARY((bits) &0x20)), (B1_TO_BINARY((bits) &0x40)), \
      (B1_TO_BINARY((bits) &0x80)), (B1_TO_BINARY((bits) &0x100)), (B1_TO_BINARY((bits) &0x200)), \
      (B1_TO_BINARY((bits) &0x400)), (B1_TO_BINARY((bits) &0x800)), (B1_TO_BINARY((bits) &0x1000)), \
      (B1_TO_BINARY((bits) &0x2000)), (B1_TO_BINARY((bits) &0x4000)), (B1_TO_BINARY((bits) &0x8000)), \
      (B1_TO_BINARY((bits) &0x10000)), (B1_TO_BINARY((bits) &0x20000)), (B1_TO_BINARY((bits) &0x40000)), \
      (B1_TO_BINARY((bits) &0x80000)), (B1_TO_BINARY((bits) &0x100000)), (B1_TO_BINARY((bits) &0x200000)), \
      (B1_TO_BINARY((bits) &0x400000)), (B1_TO_BINARY((bits) &0x800000))

#if ESP_IDF_VERSION_MAJOR <= 4

#define LOG_TMC22XX_GCONF_REGISTER_DUMP(this) \
  const int32_t gconf = this->read_register(GCONF); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B10_TO_BINARY_PATTERN, "GCONF:", gconf, B10_TO_BINARY(gconf)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "I_SCALE_ANALOG:", this->extract_field(gconf, I_SCALE_ANALOG_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "INTERNAL_RSENSE:", this->extract_field(gconf, INTERNAL_RSENSE_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "EN_SPREADCYCLE:", this->extract_field(gconf, EN_SPREADCYCLE_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SHAFT:", this->extract_field(gconf, SHAFT_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "INDEX_OTPW:", this->extract_field(gconf, INDEX_OTPW_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "INDEX_STEP:", this->extract_field(gconf, INDEX_STEP_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "PDN_DISABLE:", this->extract_field(gconf, PDN_DISABLE_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MSTEP_REG_SELECT:", this->extract_field(gconf, MSTEP_REG_SELECT_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MULTISTEP_FILT:", this->extract_field(gconf, MULTISTEP_FILT_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "TEST_MODE:", this->extract_field(gconf, TEST_MODE_FIELD));

#define LOG_TMC22XX_VERSION(this) \
  const int8_t icv_ = this->read_field(VERSION_FIELD); \
  if (icv_ == IC_VERSION_GEN1) { \
    ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", icv_); \
  } else { \
    if (icv_ == 0) { \
      ESP_LOGE(TAG, "  Unable to read IC version. Is the driver powered and wired correctly?"); \
    } else { \
      ESP_LOGE(TAG, "  Detected unknown IC version: 0x%02X", icv_); \
    } \
  }

#define LOG_TMC22XX_PINS(this) \
  if (this->enn_pin_) { \
    LOG_PIN("  ENN Pin: ", this->enn_pin_); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Enable/disable driver with TOFF"); \
  } \
  if (this->diag_pin_) { \
    LOG_PIN("  DIAG Pin: ", this->diag_pin_); \
  } \
  LOG_PIN("  INDEX Pin: ", this->index_pin_); \
  LOG_PIN("  STEP Pin: ", this->step_pin_); \
  LOG_PIN("  DIR Pin: ", this->dir_pin_);

#define LOG_TMC22XX_CURRENTS(this) \
  const bool ir_ = this->read_field(INTERNAL_RSENSE_FIELD); \
  const bool vs_ = this->read_field(VSENSE_FIELD); \
  const bool isa_ = this->read_field(I_SCALE_ANALOG_FIELD); \
  ESP_LOGCONFIG(TAG, "  Currents:"); \
  ESP_LOGCONFIG(TAG, "    Limits: %d mA", this->current_scale_to_rms_current_mA(31)); \
  ESP_LOGCONFIG(TAG, "    IRUN: %d (%d mA)", this->read_field(IRUN_FIELD), this->read_run_current_mA()); \
  ESP_LOGCONFIG(TAG, "    IHOLD: %d (%d mA)", this->read_field(IHOLD_FIELD), this->read_hold_current_mA()); \
  ESP_LOGCONFIG(TAG, "    Additional scaling by VREF is %s", (isa_ ? "enabled" : "disabled")); \
  ESP_LOGCONFIG(TAG, "    VSense: %s", (vs_ ? "True (low heat dissipation)" : "False (high heat dissipation)")); \
  if (this->rsense_.has_value()) { \
    ESP_LOGCONFIG(TAG, "    RSense: %.3f Ohm external sense resistors", this->rsense_.value()); \
  } else { \
    ESP_LOGCONFIG(TAG, "    RSense: internal RDSon"); \
  }

#define LOG_TMC22XX_GSTAT_REGISTER_DUMP(this) \
  const int32_t gstat = this->read_register(GSTAT); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B3_TO_BINARY_PATTERN, "GSTAT:", gstat, B3_TO_BINARY(gstat)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "RESET:", this->extract_field(gstat, RESET_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "DRV_ERR:", this->extract_field(gstat, DRV_ERR_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "UV_CP:", this->extract_field(gstat, UV_CP_FIELD));

#define LOG_TMC22XX_OTP_READ_REGISTER_DUMP(this) \
  const int32_t otpread = this->read_register(OTP_READ); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B24_TO_BINARY_PATTERN, "OTP_READ:", otpread, B24_TO_BINARY(otpread));

#define LOG_TMC22XX_FACTORY_CONF_REGISTER_DUMP(this) \
  const int32_t fc = this->read_register(FACTORY_CONF); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B10_TO_BINARY_PATTERN, "FACTORY_CONF:", fc, B10_TO_BINARY(fc)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "FCLKTRIM:", this->extract_field(fc, FCLKTRIM_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "OTTRIM:", this->extract_field(fc, OTTRIM_FIELD));

#define LOG_TMC22XX_IOIN_REGISTER_DUMP(this) \
  const int32_t ioin = this->read_register(IOIN); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B12_TO_BINARY_PATTERN, "IOIN:", ioin, B12_TO_BINARY(ioin)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "ENN:", this->extract_field(ioin, ENN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MS1:", this->extract_field(ioin, MS1_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MS2:", this->extract_field(ioin, MS2_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "DIAG:", this->extract_field(ioin, DIAG_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "PDN_UART:", this->extract_field(ioin, PDN_UART_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "STEP:", this->extract_field(ioin, STEP_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SPREAD_EN:", this->extract_field(ioin, SEL_A_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "DIR:", this->extract_field(ioin, DIR_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "VERSION:", this->extract_field(ioin, VERSION_FIELD));

#define LOG_TMC22XX_IHOLD_IRUN_REGISTER_DUMP(this) \
  const int32_t ihir = this->read_register(IHOLD_IRUN); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B20_TO_BINARY_PATTERN, "IHOLD_IRUN:", ihir, B20_TO_BINARY(ihir)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "IHOLD:", this->extract_field(ihir, IHOLD_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "IRUN:", this->extract_field(ihir, IRUN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "IHOLDDELAY:", this->extract_field(ihir, IHOLDDELAY_FIELD));

#define LOG_TMC22XX_TPOWERDOWN_REGISTER_DUMP(this) \
  const int32_t tpd = this->read_register(TPOWERDOWN); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B8_TO_BINARY_PATTERN, "TPOWERDOWN:", tpd, B8_TO_BINARY(tpd));

#define LOG_TMC22XX_TSTEP_REGISTER_DUMP(this) \
  const int32_t tstep = this->read_register(TSTEP); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B20_TO_BINARY_PATTERN, "TSTEP:", tstep, B20_TO_BINARY(tstep));

#define LOG_TMC22XX_TPWMTHRS_REGISTER_DUMP(this) \
  const int32_t tpwmthrs = this->read_register(TPWMTHRS); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B20_TO_BINARY_PATTERN, "TPWMTHRS:", tpwmthrs, B20_TO_BINARY(tpwmthrs));

#define LOG_TMC22XX_TCOOLTHRS_REGISTER_DUMP(this) \
  const int32_t tcoolthrs = this->read_register(TCOOLTHRS); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B20_TO_BINARY_PATTERN, "TCOOLTHRS:", tcoolthrs, B20_TO_BINARY(tpwmthrs));

#define LOG_TMC22XX_SGTHRS_REGISTER_DUMP(this) \
  const int32_t sgthrs = this->read_register(SGTHRS); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B8_TO_BINARY_PATTERN, "SGTHRS:", sgthrs, B8_TO_BINARY(sgthrs));

#define LOG_TMC22XX_COOLCONF_REGISTER_DUMP(this) \
  const int32_t coolconf = this->read_register(COOLCONF); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08X | 0b" B16_TO_BINARY_PATTERN, "COOLCONF:", coolconf, B16_TO_BINARY(coolconf)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEIMIN:", this->extract_field(coolconf, SEIMIN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEDN:", this->extract_field(coolconf, SEDN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEMAX:", this->extract_field(coolconf, SEMAX_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEUP:", this->extract_field(coolconf, SEUP_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEMIN:", this->extract_field(coolconf, SEMIN_FIELD));

#define LOG_TMC22XX_REGISTER_DUMP(this) \
  ESP_LOGCONFIG(TAG, "  Register dump:"); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "GCONF:", this->read_register(GCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "GSTAT:", this->read_register(GSTAT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IFCNT:", this->read_register(IFCNT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SLAVECONF:", this->read_register(SLAVECONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "OTP_PROG:", this->read_register(OTP_PROG)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "OTP_READ:", this->read_register(OTP_READ)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IOIN:", this->read_register(IOIN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "FACTORY_CONF:", this->read_register(FACTORY_CONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IHOLD_IRUN:", this->read_register(IHOLD_IRUN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TPOWERDOWN:", this->read_register(TPOWERDOWN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TSTEP:", this->read_register(TSTEP)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TPWMTHRS:", this->read_register(TPWMTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TCOOLTHRS:", this->read_register(TCOOLTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "VACTUAL:", this->read_register(VACTUAL)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SGTHRS:", this->read_register(SGTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SG_RESULT:", this->read_register(SG_RESULT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "COOLCONF:", this->read_register(COOLCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "MSCNT:", this->read_register(MSCNT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "MSCURACT:", this->read_register(MSCURACT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "CHOPCONF:", this->read_register(CHOPCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "DRV_STATUS:", this->read_register(DRV_STATUS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWM_CONF:", this->read_register(PWM_CONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWM_SCALE:", this->read_register(PWM_SCALE)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWM_AUTO:", this->read_register(PWM_AUTO));

#define LOG_TMC22XX(this) \
  LOG_TMC22XX_PINS(this); \
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_); \
  LOG_TMC22XX_VERSION(this); \
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps()); \
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz (VACTUAL factor: %f)", this->clk_freq_, this->clk_to_vactual_factor_); \
  const auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(OTTRIM_FIELD)); \
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot); \
  if (this->stall_detection_is_enabled_) { \
    if (this->diag_pin_ != nullptr) { \
      ESP_LOGCONFIG(TAG, "  Stall detection: DIAG interrupt raises flag"); \
    } else { \
      ESP_LOGCONFIG(TAG, "  Stall detection: poll driver for status"); \
    } \
  } else { \
    ESP_LOGCONFIG(TAG, "  Stall detection: disabled"); \
  } \
  ESP_LOGCONFIG(TAG, "  Status check: %s", (this->driver_health_check_is_enabled_ ? "enabled" : "disabled")); \
  LOG_TMC22XX_CURRENTS(this); \
  // #if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  //   LOG_TMC22XX_REGISTER_DUMP(this);
  // #endif

#else

#define LOG_TMC22XX_GCONF_REGISTER_DUMP(this) \
  const int32_t gconf = this->read_register(GCONF); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B10_TO_BINARY_PATTERN, "GCONF:", gconf, B10_TO_BINARY(gconf)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "I_SCALE_ANALOG:", this->extract_field(gconf, I_SCALE_ANALOG_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "INTERNAL_RSENSE:", this->extract_field(gconf, INTERNAL_RSENSE_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "EN_SPREADCYCLE:", this->extract_field(gconf, EN_SPREADCYCLE_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SHAFT:", this->extract_field(gconf, SHAFT_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "INDEX_OTPW:", this->extract_field(gconf, INDEX_OTPW_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "INDEX_STEP:", this->extract_field(gconf, INDEX_STEP_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "PDN_DISABLE:", this->extract_field(gconf, PDN_DISABLE_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MSTEP_REG_SELECT:", this->extract_field(gconf, MSTEP_REG_SELECT_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MULTISTEP_FILT:", this->extract_field(gconf, MULTISTEP_FILT_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "TEST_MODE:", this->extract_field(gconf, TEST_MODE_FIELD));

#define LOG_TMC22XX_VERSION(this) \
  const int8_t icv_ = this->read_field(VERSION_FIELD); \
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", icv_);

#define LOG_TMC22XX_PINS(this) \
  if (this->enn_pin_) { \
    LOG_PIN("  ENN Pin: ", this->enn_pin_); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Enable/disable driver with TOFF"); \
  } \
  if (this->diag_pin_) { \
    LOG_PIN("  DIAG Pin: ", this->diag_pin_); \
  } \
  LOG_PIN("  INDEX Pin: ", this->index_pin_); \
  LOG_PIN("  STEP Pin: ", this->step_pin_); \
  LOG_PIN("  DIR Pin: ", this->dir_pin_);

#define LOG_TMC22XX_CURRENTS(this) \
  const bool ir_ = this->read_field(INTERNAL_RSENSE_FIELD); \
  const bool vs_ = this->read_field(VSENSE_FIELD); \
  const bool isa_ = this->read_field(I_SCALE_ANALOG_FIELD); \
  ESP_LOGCONFIG(TAG, "  Currents:"); \
  ESP_LOGCONFIG(TAG, "    Limits: %d mA", this->current_scale_to_rms_current_mA(31)); \
  ESP_LOGCONFIG(TAG, "    IRUN: %ld (%d mA)", this->read_field(IRUN_FIELD), this->read_run_current_mA()); \
  ESP_LOGCONFIG(TAG, "    IHOLD: %ld (%d mA)", this->read_field(IHOLD_FIELD), this->read_hold_current_mA()); \
  ESP_LOGCONFIG(TAG, "    Additional scaling by VREF is %s", (isa_ ? "enabled" : "disabled")); \
  ESP_LOGCONFIG(TAG, "    VSense: %s", (vs_ ? "True (low heat dissipation)" : "False (high heat dissipation)")); \
  if (this->rsense_.has_value()) { \
    ESP_LOGCONFIG(TAG, "    RSense: %.3f Ohm external sense resistors", this->rsense_.value()); \
  } else { \
    ESP_LOGCONFIG(TAG, "    RSense: internal RDSon"); \
  }

#define LOG_TMC22XX_GSTAT_REGISTER_DUMP(this) \
  const int32_t gstat = this->read_register(GSTAT); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B3_TO_BINARY_PATTERN, "GSTAT:", gstat, B3_TO_BINARY(gstat)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "RESET:", this->extract_field(gstat, RESET_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "DRV_ERR:", this->extract_field(gstat, DRV_ERR_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "UV_CP:", this->extract_field(gstat, UV_CP_FIELD));

#define LOG_TMC22XX_OTP_READ_REGISTER_DUMP(this) \
  const int32_t otpread = this->read_register(OTP_READ); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B24_TO_BINARY_PATTERN, "OTP_READ:", otpread, B24_TO_BINARY(otpread));

#define LOG_TMC22XX_FACTORY_CONF_REGISTER_DUMP(this) \
  const int32_t fc = this->read_register(FACTORY_CONF); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B10_TO_BINARY_PATTERN, "FACTORY_CONF:", fc, B10_TO_BINARY(fc)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "FCLKTRIM:", this->extract_field(fc, FCLKTRIM_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "OTTRIM:", this->extract_field(fc, OTTRIM_FIELD));

#define LOG_TMC22XX_IOIN_REGISTER_DUMP(this) \
  const int32_t ioin = this->read_register(IOIN); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B12_TO_BINARY_PATTERN, "IOIN:", ioin, B12_TO_BINARY(ioin)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "ENN:", this->extract_field(ioin, ENN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MS1:", this->extract_field(ioin, MS1_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "MS2:", this->extract_field(ioin, MS2_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "DIAG:", this->extract_field(ioin, DIAG_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "PDN_UART:", this->extract_field(ioin, PDN_UART_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "STEP:", this->extract_field(ioin, STEP_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SPREAD_EN:", this->extract_field(ioin, SEL_A_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "DIR:", this->extract_field(ioin, DIR_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "VERSION:", this->extract_field(ioin, VERSION_FIELD));

#define LOG_TMC22XX_IHOLD_IRUN_REGISTER_DUMP(this) \
  const int32_t ihir = this->read_register(IHOLD_IRUN); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B20_TO_BINARY_PATTERN, "IHOLD_IRUN:", ihir, B20_TO_BINARY(ihir)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "IHOLD:", this->extract_field(ihir, IHOLD_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "IRUN:", this->extract_field(ihir, IRUN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "IHOLDDELAY:", this->extract_field(ihir, IHOLDDELAY_FIELD));

#define LOG_TMC22XX_TPOWERDOWN_REGISTER_DUMP(this) \
  const int32_t tpd = this->read_register(TPOWERDOWN); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B8_TO_BINARY_PATTERN, "TPOWERDOWN:", tpd, B8_TO_BINARY(tpd));

#define LOG_TMC22XX_TSTEP_REGISTER_DUMP(this) \
  const int32_t tstep = this->read_register(TSTEP); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B20_TO_BINARY_PATTERN, "TSTEP:", tstep, B20_TO_BINARY(tstep));

#define LOG_TMC22XX_TPWMTHRS_REGISTER_DUMP(this) \
  const int32_t tpwmthrs = this->read_register(TPWMTHRS); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B20_TO_BINARY_PATTERN, "TPWMTHRS:", tpwmthrs, B20_TO_BINARY(tpwmthrs));

#define LOG_TMC22XX_TCOOLTHRS_REGISTER_DUMP(this) \
  const int32_t tcoolthrs = this->read_register(TCOOLTHRS); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B20_TO_BINARY_PATTERN, "TCOOLTHRS:", tcoolthrs, B20_TO_BINARY(tpwmthrs));

#define LOG_TMC22XX_SGTHRS_REGISTER_DUMP(this) \
  const int32_t sgthrs = this->read_register(SGTHRS); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B8_TO_BINARY_PATTERN, "SGTHRS:", sgthrs, B8_TO_BINARY(sgthrs));

#define LOG_TMC22XX_COOLCONF_REGISTER_DUMP(this) \
  const int32_t coolconf = this->read_register(COOLCONF); \
  ESP_LOGCONFIG(TAG, "   %-18s 0x%08lX | 0b" B16_TO_BINARY_PATTERN, "COOLCONF:", coolconf, B16_TO_BINARY(coolconf)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEIMIN:", this->extract_field(coolconf, SEIMIN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEDN:", this->extract_field(coolconf, SEDN_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEMAX:", this->extract_field(coolconf, SEMAX_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEUP:", this->extract_field(coolconf, SEUP_FIELD)); \
  ESP_LOGCONFIG(TAG, "    %-18s 0x%x", "SEMIN:", this->extract_field(coolconf, SEMIN_FIELD));

#define LOG_TMC22XX_REGISTER_DUMP(this) \
  ESP_LOGCONFIG(TAG, "  Register dump:"); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "GCONF:", this->read_register(GCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "GSTAT:", this->read_register(GSTAT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "IFCNT:", this->read_register(IFCNT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "SLAVECONF:", this->read_register(SLAVECONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "OTP_PROG:", this->read_register(OTP_PROG)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "OTP_READ:", this->read_register(OTP_READ)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "IOIN:", this->read_register(IOIN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "FACTORY_CONF:", this->read_register(FACTORY_CONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "IHOLD_IRUN:", this->read_register(IHOLD_IRUN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "TPOWERDOWN:", this->read_register(TPOWERDOWN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "TSTEP:", this->read_register(TSTEP)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "TPWMTHRS:", this->read_register(TPWMTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "TCOOLTHRS:", this->read_register(TCOOLTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "VACTUAL:", this->read_register(VACTUAL)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "SGTHRS:", this->read_register(SGTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "SG_RESULT:", this->read_register(SG_RESULT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "COOLCONF:", this->read_register(COOLCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "MSCNT:", this->read_register(MSCNT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "MSCURACT:", this->read_register(MSCURACT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "CHOPCONF:", this->read_register(CHOPCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "DRV_STATUS:", this->read_register(DRV_STATUS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "PWM_CONF:", this->read_register(PWM_CONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "PWM_SCALE:", this->read_register(PWM_SCALE)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08lX", "PWM_AUTO:", this->read_register(PWM_AUTO));

#define LOG_TMC22XX(this) \
  LOG_TMC22XX_PINS(this); \
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_); \
  LOG_TMC22XX_VERSION(this); \
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps()); \
  ESP_LOGCONFIG(TAG, "  Clock frequency: %ld Hz (VACTUAL factor: %f)", this->clk_freq_, this->clk_to_vactual_factor_); \
  const auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(OTTRIM_FIELD)); \
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot); \
  if (this->stall_detection_is_enabled_) { \
    if (this->diag_pin_ != nullptr) { \
      ESP_LOGCONFIG(TAG, "  Stall detection: DIAG interrupt raises flag"); \
    } else { \
      ESP_LOGCONFIG(TAG, "  Stall detection: poll driver for status"); \
    } \
  } else { \
    ESP_LOGCONFIG(TAG, "  Stall detection: disabled"); \
  } \
  ESP_LOGCONFIG(TAG, "  Status check: %s", (this->driver_health_check_is_enabled_ ? "enabled" : "disabled")); \
  LOG_TMC22XX_CURRENTS(this); \
  // #if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  //   LOG_TMC22XX_REGISTER_DUMP(this);
  // #endif

#endif

}  // namespace tmc22xx
}  // namespace esphome
