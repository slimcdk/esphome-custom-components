#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint16_t TMC2209::read_gconf() { return tmc2209_readInt(&this->driver_, TMC2209_GCONF); }
void TMC2209::write_gconf(uint16_t setting) { return tmc2209_writeInt(&this->driver_, TMC2209_GCONF, setting); }

void TMC2209::read_gconf_update() {
  this->gconf_ = this->read_gconf();
  this->gconf_last_read_ = (time_t) millis();
}

void TMC2209::write_gconf_update() {
  this->write_gconf(this->gconf_);
  this->gconf_last_write_ = (time_t) millis();
}

void TMC2209::write_iscale_analog(bool use_vref) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_I_SCALE_ANALOG_MASK, TMC2209_I_SCALE_ANALOG_SHIFT,
                       use_vref);
}

bool TMC2209::read_iscale_analog() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_I_SCALE_ANALOG_MASK, TMC2209_I_SCALE_ANALOG_SHIFT);
}

void TMC2209::write_internal_rsense(bool use_internal) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_INTERNAL_RSENSE_MASK, TMC2209_INTERNAL_RSENSE_SHIFT,
                       use_internal);
}

bool TMC2209::read_internal_rsense() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_INTERNAL_RSENSE_MASK, TMC2209_INTERNAL_RSENSE_SHIFT);
}

void TMC2209::write_en_spreadcycle(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_EN_SPREADCYCLE_MASK, TMC2209_EN_SPREADCYCLE_SHIFT,
                       enable);
}

bool TMC2209::read_en_spreadcycle() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_EN_SPREADCYCLE_MASK, TMC2209_EN_SPREADCYCLE_SHIFT);
}

bool TMC2209::read_inverse_direction() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT);
}

void TMC2209::write_inverse_direction(bool inverse_direction) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, inverse_direction);
}

bool TMC2209::read_index_otpw() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_OTPW_MASK, TMC2209_INDEX_OTPW_SHIFT);
}

void TMC2209::write_index_otpw(bool use_for_otpw) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_OTPW_MASK, TMC2209_INDEX_OTPW_SHIFT, use_for_otpw);
}

void TMC2209::write_index_step(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_STEP_MASK, TMC2209_INDEX_STEP_SHIFT, enable);
}

bool TMC2209::read_read_index_step() {
  return (bool) TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_STEP_MASK, TMC2209_INDEX_STEP_SHIFT);
}

void TMC2209::write_pdn_disable(bool disable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, disable);
}

bool TMC2209::read_pdn_disable() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT);
}

void TMC2209::write_use_mres_register(bool use) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_MSTEP_REG_SELECT_MASK, TMC2209_MSTEP_REG_SELECT_SHIFT,
                       use);
}

bool TMC2209::read_use_mres_register() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_MSTEP_REG_SELECT_MASK,
                            TMC2209_MSTEP_REG_SELECT_SHIFT);
}

void TMC2209::write_multistep_filt(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_MULTISTEP_FILT_MASK, TMC2209_MULTISTEP_FILT_SHIFT,
                       enable);
}

bool TMC2209::read_multistep_filt() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_MULTISTEP_FILT_MASK, TMC2209_MULTISTEP_FILT_SHIFT);
}

void TMC2209::write_test_mode(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_TEST_MODE_MASK, TMC2209_TEST_MODE_SHIFT, enable);
}

bool TMC2209::read_test_mode() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_TEST_MODE_MASK, TMC2209_TEST_MODE_SHIFT);
}

/** Shadow registers **/

void TMC2209::set_iscale_analog(bool use_vref) { return this->write_iscale_analog(use_vref); }
void TMC2209::set_internal_rsense(bool use_internal) {
  return this->write_internal_rsense(use_internal);  // TODO: use shadow register
}
void TMC2209::set_en_spreadcycle(bool enable) {
  return this->write_en_spreadcycle(enable);  // TODO: use shadow register
}
void TMC2209::set_inverse_direction(bool inverse_direction) {
  return this->write_inverse_direction(inverse_direction);  // TODO: use shadow register
}
void TMC2209::set_index_otpw(bool use_for_otpw) {
  return this->write_index_otpw(use_for_otpw);  // TODO: use shadow register
}
void TMC2209::set_index_step(bool enable) { return this->write_index_step(enable); }  // TODO: use shadow register
void TMC2209::set_use_mres_register(bool use) {
  return this->write_use_mres_register(use);  // TODO: use shadow register
}
void TMC2209::set_multistep_filt(bool enable) {
  return this->write_multistep_filt(enable);  // TODO: use shadow register
}
void TMC2209::set_pdn_disable(bool disable) { return this->write_pdn_disable(disable); }  // TODO: use shadow register
void TMC2209::set_test_mode(bool enable) { this->write_test_mode(enable); }               // TODO: use shadow register

bool TMC2209::get_inverse_direction() { return this->read_inverse_direction(); }  // TODO: use shadow register
bool TMC2209::get_internal_rsense() { return this->read_internal_rsense(); }      // TODO: use shadow register
bool TMC2209::get_en_spreadcycle() { return this->read_en_spreadcycle(); }        // TODO: use shadow register
bool TMC2209::get_index_otpw() { return this->read_index_otpw(); }                // TODO: use shadow register
bool TMC2209::get_read_index_step() { return this->read_read_index_step(); }      // TODO: use shadow register
bool TMC2209::get_pdn_disable() { return this->read_pdn_disable(); }              // TODO: use shadow register
bool TMC2209::get_use_mres_register() { return this->read_use_mres_register(); }  // TODO: use shadow register
bool TMC2209::get_multistep_filt() { return this->read_multistep_filt(); }        // TODO: use shadow register
bool TMC2209::get_iscale_analog() { return this->read_iscale_analog(); }          // TODO: use shadow register
bool TMC2209::get_test_mode() { return this->read_test_mode(); }                  // TODO: use shadow register

}  // namespace tmc
}  // namespace esphome
