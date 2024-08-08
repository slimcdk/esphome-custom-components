#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc2209.h"

namespace esphome {
namespace tmc {

/***
 * TMC-API wrappers
 *
 **/
/*
extern "C" {
bool tmc2209_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209Stepper *comp = components[id];

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    comp->read_array(data, readLength);
  }
  return true;
}

uint8_t tmc2209_getNodeAddress(uint16_t id) {
  TMC2209Stepper *comp = components[id];
  return comp->address_;
}
}

uint16_t TMC2209Stepper::internal_step_counter() { return tmc2209_fieldRead(this->id_, TMC2209_MSCNT_FIELD); };
int16_t TMC2209Stepper::current_a() { return tmc2209_fieldRead(this->id_, TMC2209_CUR_A_FIELD); };
int16_t TMC2209Stepper::current_b() { return tmc2209_fieldRead(this->id_, TMC2209_CUR_B_FIELD); };
void TMC2209Stepper::vactual(int32_t velocity) { tmc2209_fieldWrite(this->id_, TMC2209_VACTUAL_FIELD, velocity); }
uint32_t TMC2209Stepper::tstep() { return tmc2209_fieldRead(this->id_, TMC2209_TSTEP_FIELD); }
void TMC2209Stepper::ihold_irun_ihold(uint8_t current) { tmc2209_fieldWrite(this->id_, TMC2209_IRUN_FIELD, current); }
uint8_t TMC2209Stepper::ihold_irun_ihold() { return tmc2209_fieldRead(this->id_, TMC2209_IRUN_FIELD); }
void TMC2209Stepper::ihold_irun_irun(uint8_t current) { tmc2209_fieldWrite(this->id_, TMC2209_IHOLD_FIELD, current); }
uint8_t TMC2209Stepper::ihold_irun_irun() { return tmc2209_fieldRead(this->id_, TMC2209_IHOLD_FIELD); }
void TMC2209Stepper::ihold_irun_ihold_delay(uint8_t factor) {
  if (factor > 15) {
    ESP_LOGW(TAG, "IHOLDDELAY is limited to 15. This is the raw value and not the delay in microseconds.");
    factor = 15;
  }
  tmc2209_fieldWrite(this->id_, TMC2209_IHOLDDELAY_FIELD, factor);
}
uint8_t TMC2209Stepper::ihold_irun_ihold_delay() { return tmc2209_fieldRead(this->id_, TMC2209_IHOLDDELAY_FIELD); }
uint32_t TMC2209Stepper::otpread() { return tmc2209_readRegister(this->id_, TMC2209_OTP_READ); }
bool TMC2209Stepper::optread_en_spreadcycle() { return (bool) tmc2209_fieldRead(this->id_, TMC2209_STST_FIELD); }
void TMC2209Stepper::stallguard_sgthrs(uint8_t threshold) {
  tmc2209_writeRegister(this->id_, TMC2209_SGTHRS, (int32_t) threshold);
  this->stallguard_sgthrs_ = threshold;
}
uint8_t TMC2209Stepper::stallguard_sgthrs() { return this->stallguard_sgthrs_; }
uint16_t TMC2209Stepper::stallguard_sgresult() { return (uint16_t) tmc2209_readRegister(this->id_, TMC2209_SG_RESULT); }
uint32_t TMC2209Stepper::ioin() { return tmc2209_readRegister(this->id_, TMC2209_IOIN); }
bool TMC2209Stepper::ioin_enn() { return tmc2209_fieldRead(this->id_, TMC2209_ENN_FIELD); }
bool TMC2209Stepper::ioin_ms1() { return tmc2209_fieldRead(this->id_, TMC2209_MS1_FIELD); }
bool TMC2209Stepper::ioin_ms2() { return tmc2209_fieldRead(this->id_, TMC2209_MS2_FIELD); }
bool TMC2209Stepper::ioin_diag() { return tmc2209_fieldRead(this->id_, TMC2209_DIAG_FIELD); }
bool TMC2209Stepper::ioin_pdn_uart() { return tmc2209_fieldRead(this->id_, TMC2209_PDN_UART_FIELD); }
bool TMC2209Stepper::ioin_step() { return tmc2209_fieldRead(this->id_, TMC2209_STEP_FIELD); }
bool TMC2209Stepper::ioin_spread_en() { return tmc2209_fieldRead(this->id_, TMC2209_SEL_A_FIELD); }
bool TMC2209Stepper::ioin_dir() { return tmc2209_fieldRead(this->id_, TMC2209_DIR_FIELD); }
int8_t TMC2209Stepper::ioin_chip_version() { return tmc2209_fieldRead(this->id_, TMC2209_VERSION_FIELD); }
uint8_t TMC2209Stepper::transmission_counter() { return tmc2209_fieldRead(this->id_, TMC2209_IFCNT_FIELD); }
uint16_t TMC2209Stepper::gstat() { return tmc2209_readRegister(this->id_, TMC2209_GSTAT); }
void TMC2209Stepper::gstat(uint16_t setting) { return tmc2209_writeRegister(this->id_, TMC2209_GSTAT, setting); }
bool TMC2209Stepper::gstat_reset() { return tmc2209_fieldRead(this->id_, TMC2209_RESET_FIELD); }
void TMC2209Stepper::gstat_reset(bool clear) { tmc2209_fieldWrite(this->id_, TMC2209_RESET_FIELD, clear); }
bool TMC2209Stepper::gstat_drv_err() { return tmc2209_fieldRead(this->id_, TMC2209_DRV_ERR_FIELD); }
void TMC2209Stepper::gstat_drv_err(bool clear) { tmc2209_fieldWrite(this->id_, TMC2209_DRV_ERR_FIELD, clear); }
bool TMC2209Stepper::gstat_uv_cp() { return tmc2209_fieldRead(this->id_, TMC2209_UV_CP_FIELD); }
void TMC2209Stepper::gstat_uv_cp(bool clear) { tmc2209_fieldWrite(this->id_, TMC2209_UV_CP_FIELD, clear); }
uint16_t TMC2209Stepper::gconf() { return tmc2209_readRegister(this->id_, TMC2209_GCONF); }
void TMC2209Stepper::gconf(uint16_t setting) { return tmc2209_writeRegister(this->id_, TMC2209_GCONF, setting); }
void TMC2209Stepper::gconf_iscale_analog(bool use_vref) {
  tmc2209_fieldWrite(this->id_, TMC2209_I_SCALE_ANALOG_FIELD, use_vref);
}
bool TMC2209Stepper::gconf_iscale_analog() { return tmc2209_fieldRead(this->id_, TMC2209_I_SCALE_ANALOG_FIELD); }
void TMC2209Stepper::gconf_internal_rsense(bool use_internal) {
  tmc2209_fieldWrite(this->id_, TMC2209_INTERNAL_RSENSE_FIELD, use_internal);
}
bool TMC2209Stepper::gconf_internal_rsense() { return tmc2209_fieldRead(this->id_, TMC2209_INTERNAL_RSENSE_FIELD); }
void TMC2209Stepper::gconf_en_spreadcycle(bool enable) {
  tmc2209_fieldWrite(this->id_, TMC2209_EN_SPREADCYCLE_FIELD, enable);
}
bool TMC2209Stepper::gconf_en_spreadcycle() { return tmc2209_fieldRead(this->id_, TMC2209_EN_SPREADCYCLE_FIELD); }
bool TMC2209Stepper::gconf_shaft() { return tmc2209_fieldRead(this->id_, TMC2209_SHAFT_FIELD); }
void TMC2209Stepper::gconf_shaft(bool inverse) { tmc2209_fieldWrite(this->id_, TMC2209_SHAFT_FIELD, inverse); }
bool TMC2209Stepper::gconf_index_otpw() { return tmc2209_fieldRead(this->id_, TMC2209_INDEX_OTPW_FIELD); }
void TMC2209Stepper::gconf_index_otpw(bool use_otpw) {
  tmc2209_fieldWrite(this->id_, TMC2209_INDEX_OTPW_FIELD, use_otpw);
}
void TMC2209Stepper::gconf_index_step(bool enable) { tmc2209_fieldWrite(this->id_, TMC2209_INDEX_STEP_FIELD, enable); }
bool TMC2209Stepper::gconf_index_step() { return (bool) tmc2209_fieldRead(this->id_, TMC2209_INDEX_STEP_FIELD); }
void TMC2209Stepper::gconf_pdn_disable(bool disable) {
  tmc2209_fieldWrite(this->id_, TMC2209_PDN_DISABLE_FIELD, disable);
}
bool TMC2209Stepper::gconf_pdn_disable() { return tmc2209_fieldRead(this->id_, TMC2209_PDN_DISABLE_FIELD); }
void TMC2209Stepper::gconf_mstep_reg_select(bool use) {
  tmc2209_fieldWrite(this->id_, TMC2209_MSTEP_REG_SELECT_FIELD, use);
}
bool TMC2209Stepper::gconf_mstep_reg_select() { return tmc2209_fieldRead(this->id_, TMC2209_MSTEP_REG_SELECT_FIELD); }
void TMC2209Stepper::gconf_multistep_filt(bool enable) {
  tmc2209_fieldWrite(this->id_, TMC2209_MULTISTEP_FILT_FIELD, enable);
}
bool TMC2209Stepper::gconf_multistep_filt() { return tmc2209_fieldRead(this->id_, TMC2209_MULTISTEP_FILT_FIELD); }
void TMC2209Stepper::gconf_test_mode(bool enable) { tmc2209_fieldWrite(this->id_, TMC2209_TEST_MODE_FIELD, enable); }
bool TMC2209Stepper::gconf_test_mode() { return tmc2209_fieldRead(this->id_, TMC2209_TEST_MODE_FIELD); }
void TMC2209Stepper::fclktrim(uint8_t fclktrim) { tmc2209_fieldWrite(this->id_, TMC2209_FCLKTRIM_FIELD, fclktrim); }
uint8_t TMC2209Stepper::fclktrim() { return tmc2209_fieldRead(this->id_, TMC2209_FCLKTRIM_FIELD); }
void TMC2209Stepper::ottrim(uint8_t ottrim) { tmc2209_fieldWrite(this->id_, TMC2209_OTTRIM_FIELD, ottrim); }
uint8_t TMC2209Stepper::ottrim() { return tmc2209_fieldRead(this->id_, TMC2209_OTTRIM_FIELD); }
bool TMC2209Stepper::drv_status_stst() { return tmc2209_fieldRead(this->id_, TMC2209_STST_FIELD); }
bool TMC2209Stepper::drv_status_stealth() { return tmc2209_fieldRead(this->id_, TMC2209_STEALTH_FIELD); }
uint8_t TMC2209Stepper::drv_status_cs_actual() { return tmc2209_fieldRead(this->id_, TMC2209_CS_ACTUAL_FIELD); }
bool TMC2209Stepper::drv_status_otpw() { return tmc2209_fieldRead(this->id_, TMC2209_OTPW_FIELD); }
bool TMC2209Stepper::drv_status_ot() { return tmc2209_fieldRead(this->id_, TMC2209_OT_FIELD); }
bool TMC2209Stepper::drv_status_t120() { return tmc2209_fieldRead(this->id_, TMC2209_T120_FIELD); }
bool TMC2209Stepper::drv_status_t143() { return tmc2209_fieldRead(this->id_, TMC2209_T143_FIELD); }
bool TMC2209Stepper::drv_status_t150() { return tmc2209_fieldRead(this->id_, TMC2209_T150_FIELD); }
bool TMC2209Stepper::drv_status_t157() { return tmc2209_fieldRead(this->id_, TMC2209_T157_FIELD); }
bool TMC2209Stepper::drv_status_ola() { return tmc2209_fieldRead(this->id_, TMC2209_OLA_FIELD); }
bool TMC2209Stepper::drv_status_olb() { return tmc2209_fieldRead(this->id_, TMC2209_OLB_FIELD); }
bool TMC2209Stepper::drv_status_s2vsa() { return tmc2209_fieldRead(this->id_, TMC2209_S2VSA_FIELD); }
bool TMC2209Stepper::drv_status_s2vsb() { return tmc2209_fieldRead(this->id_, TMC2209_S2VSB_FIELD); }
bool TMC2209Stepper::drv_status_s2ga() { return tmc2209_fieldRead(this->id_, TMC2209_S2GA_FIELD); }
bool TMC2209Stepper::drv_status_s2gb() { return tmc2209_fieldRead(this->id_, TMC2209_S2GB_FIELD); }
void TMC2209Stepper::coolstep_tcoolthrs(int32_t threshold) {
  tmc2209_writeRegister(this->id_, TMC2209_TCOOLTHRS, threshold);
  this->coolstep_tcoolthrs_ = threshold;
}
int32_t TMC2209Stepper::coolstep_tcoolthrs() { return this->coolstep_tcoolthrs_; }
void TMC2209Stepper::blank_time(uint8_t select) { tmc2209_fieldWrite(this->id_, TMC2209_TBL_FIELD, select); }
void TMC2209Stepper::chopconf_mres(uint8_t index) { tmc2209_fieldWrite(this->id_, TMC2209_MRES_FIELD, index); }
uint8_t TMC2209Stepper::chopconf_mres() { return tmc2209_fieldRead(this->id_, TMC2209_MRES_FIELD); }
bool TMC2209Stepper::chopconf_dedge() { return tmc2209_fieldRead(this->id_, TMC2209_DEDGE_FIELD); }
void TMC2209Stepper::chopconf_dedge(bool set) { tmc2209_fieldWrite(this->id_, TMC2209_DEDGE_FIELD, set); }
void TMC2209Stepper::chopconf_vsense(bool high_sensitivity) {
  tmc2209_fieldWrite(this->id_, TMC2209_VSENSE_FIELD, high_sensitivity);
}
bool TMC2209Stepper::chopconf_vsense() { return tmc2209_fieldRead(this->id_, TMC2209_VSENSE_FIELD); }
void TMC2209Stepper::tpowerdown(uint8_t factor) { tmc2209_fieldWrite(this->id_, TMC2209_TPOWERDOWN_FIELD, factor); }
uint8_t TMC2209Stepper::tpowerdown() { return tmc2209_fieldRead(this->id_, TMC2209_TPOWERDOWN_FIELD); }
*/
}  // namespace tmc
}  // namespace esphome
