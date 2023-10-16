

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc2300.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2300.stepper";

void IRAM_ATTR HOT TMC2300ISRStore::index_isr(TMC2300ISRStore *arg) {
  (*(arg->current_position_ptr)) += *(arg->direction_ptr);
  if (*(arg->current_position_ptr) == *(arg->target_position_ptr)) {
    arg->stop();
  }
}

void IRAM_ATTR HOT TMC2300ISRStore::stop() {
  this->enn_pin_.digital_write(false);
  *(this->driver_is_enabled_ptr) = false;
  *(this->direction_ptr) = Direction::NONE;  // TODO: maybe remove
  *(this->target_position_ptr) = *(this->current_position_ptr);
}

// Global list of driver to work around TMC channel index
TMC2300Stepper::TMC2300Stepper(uint8_t address, bool use_internal_rsense, float resistance)
    : address_(address), use_internal_rsense_(use_internal_rsense), rsense_(resistance) {
  this->channel_ = tmc2300_stepper_global_index++;
  components[this->channel_] = this;

  // Initialize TMC-API object
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2300_init(&this->driver_, this->channel_, &this->config_, &tmc2300_defaultRegisterResetState[0]);
  tmc2300_setSlaveAddress(&this->driver_, this->address_);
  tmc2300_setStandby(&this->driver_, false);
}

void TMC2300Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2300...");

  this->enn_pin_->setup();
  this->diag_pin_->setup();

  // Fill default configuration for driver (TODO: retry call instead)
  if (!tmc2300_reset(&this->driver_)) {
    this->mark_failed();
    return;
  }

  for (uint8_t i = 0; i < 255 && this->driver_.config->state != CONFIG_READY; i++) {
    this->update_registers_();  // Write initial configuration
  }

  if (this->driver_.config->state != CONFIG_READY) {
    ESP_LOGE(TAG, "Failed to communicate with the driver");
    this->mark_failed();
    return;
  }

  this->gconf_pdn_disable(true);       // Prioritize UART communication by disabling configuration pin.
  this->gconf_mstep_reg_select(true);  // Use MSTEP register to set microstep resolution
  // Configure stepping pulses on index pin
  this->gconf_diag_index(true);
  this->gconf_diag_step(true);

  // Pulse tracker setup
  this->isr_store_.enn_pin_ = this->enn_pin_->to_isr();
  this->isr_store_.target_position_ptr = &this->target_position;
  this->isr_store_.current_position_ptr = &this->current_position;
  this->isr_store_.direction_ptr = &this->direction_;
  this->isr_store_.driver_is_enabled_ptr = &this->driver_is_enabled_;

  this->diag_pin_->attach_interrupt(TMC2300ISRStore::index_isr, &this->isr_store_, gpio::INTERRUPT_RISING_EDGE);
  ESP_LOGCONFIG(TAG, "TMC2300 Stepper setup done.");
}

void TMC2300Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2300 Stepper:");
  LOG_PIN("  Enn Pin: ", this->enn_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  ESP_LOGCONFIG(TAG, "  RSense: %.2f Ohm (%s)", this->rsense_, this->use_internal_rsense_ ? "Internal" : "External");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X (TMCAPI Channel: %d)", this->address_, this->channel_);
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", this->ioin_version());
  LOG_STEPPER(this);
}

void TMC2300Stepper::loop() {
  if (this->ioin_diag()) {
    this->stop();
    this->on_stall_callback_.call();
  }

  const bool has_reached_target_ = this->has_reached_target();
  has_reached_target_ ? this->high_freq_.stop() : this->high_freq_.start();

  this->enable(!has_reached_target_);

  const int32_t to_target = (this->target_position - this->current_position);
  this->direction_ = (Direction) (to_target != 0 ? to_target / abs(to_target) : NONE);  // yield 1, -1 or 0
  this->calculate_speed_(micros());
  this->vactual(this->direction_ * (int32_t) this->current_speed_);  // TODO: write only when values change
  this->update_registers_();
}

void TMC2300Stepper::stop() {
  this->direction_ = Direction::NONE;
  this->target_position = this->current_position;
  this->vactual(0);
  this->enable(false);
}

void TMC2300Stepper::enable(bool enable) {
  this->enn_pin_->digital_write(enable);
  this->driver_is_enabled_ = enable;
}

bool TMC2300Stepper::reset_() { return tmc2300_reset(&this->driver_); }
bool TMC2300Stepper::restore_() { return tmc2300_restore(&this->driver_); }
void TMC2300Stepper::update_registers_() { return tmc2300_periodicJob(&this->driver_, 0); }

uint16_t TMC2300Stepper::get_microsteps() {
  const uint8_t mres = this->chopconf_mres();
  return 256 >> mres;
}

void TMC2300Stepper::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->chopconf_mres(mres);
}

float TMC2300Stepper::motor_load() {
  const uint16_t result = this->stallguard_sgresult();
  return (510.0 - (float) result) / (510.0 - (float) this->stallguard_sgthrs_ * 2.0);
}

float TMC2300Stepper::rms_current_hold_scale() { return this->rms_current_hold_scale_; }
void TMC2300Stepper::rms_current_hold_scale(float scale) {
  this->rms_current_hold_scale_ = scale;
  this->set_rms_current_();
}

void TMC2300Stepper::rms_current(uint16_t mA) {
  this->rms_current_ = mA;
  this->set_rms_current_();
}

uint16_t TMC2300Stepper::rms_current() { return this->current_scale_to_rms_current_(this->ihold_irun_irun()); }

void TMC2300Stepper::set_rms_current_() {
  uint8_t current_scale = 32.0 * 1.41421 * this->rms_current_ / 1000.0 * (this->rsense_ + 0.02) / 0.325 - 1;

  if (current_scale > 31) {
    current_scale = 31;
    ESP_LOGW(TAG, "Selected rsense has a current limit of %d mA", this->current_scale_to_rms_current_(current_scale));
  }

  this->ihold_irun_irun(current_scale);
  this->ihold_irun_ihold(current_scale * this->rms_current_hold_scale_);
}

uint16_t TMC2300Stepper::current_scale_to_rms_current_(uint8_t current_scaling) {
  return (float) (current_scaling + 1) / 32.0 * 0.325 / (this->rsense_ + 0.02) / 1.41421 * 1000;
}

/**
 * TMC API set-/getters
 *
 * **/

extern "C" {
void tmc2300_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2300Stepper *comp = components[channel];

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    comp->read_array(data, readLength);
  }
}

uint8_t tmc2300_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }
}

uint16_t TMC2300Stepper::internal_step_counter() { return (uint16_t) tmc2300_readInt(&this->driver_, TMC2300_MSCNT); };

void TMC2300Stepper::vactual(int32_t velocity) {
  TMC2300_FIELD_WRITE(&this->driver_, TMC2300_VACTUAL, TMC2300_VACTUAL_MASK, TMC2300_VACTUAL_SHIFT, velocity);
}

void TMC2300Stepper::ihold_irun_ihold(uint8_t current) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IRUN_MASK, TMC2300_IRUN_SHIFT, current);
}

uint8_t TMC2300Stepper::ihold_irun_ihold() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IRUN_MASK, TMC2300_IRUN_SHIFT);
}

void TMC2300Stepper::ihold_irun_irun(uint8_t current) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IHOLD_MASK, TMC2300_IHOLD_SHIFT, current);
}

uint8_t TMC2300Stepper::ihold_irun_irun() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IHOLD_MASK, TMC2300_IHOLD_SHIFT);
}

void TMC2300Stepper::ihold_irun_ihold_delay(uint8_t delay) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IHOLDDELAY_MASK, TMC2300_IHOLDDELAY_SHIFT, delay);
}
uint8_t TMC2300Stepper::ihold_irun_ihold_delay() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IHOLDDELAY_MASK, TMC2300_IHOLDDELAY_SHIFT);
}

void TMC2300Stepper::stallguard_sgthrs(uint8_t threshold) {
  tmc2300_writeInt(&this->driver_, TMC2300_SGTHRS, (int32_t) threshold);
  this->stallguard_sgthrs_ = threshold;
}

uint8_t TMC2300Stepper::stallguard_sgthrs() { return this->stallguard_sgthrs_; }

uint16_t TMC2300Stepper::stallguard_sgresult() { return (uint16_t) tmc2300_readInt(&this->driver_, TMC2300_SG_VALUE); }

uint32_t TMC2300Stepper::ioin() { return tmc2300_readInt(&this->driver_, TMC2300_IOIN); }

bool TMC2300Stepper::ioin_en() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_EN_MASK, TMC2300_EN_SHIFT);
}

bool TMC2300Stepper::ioin_nstdby() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_NSTDBY_MASK, TMC2300_NSTDBY_SHIFT);
}

bool TMC2300Stepper::ioin_diag() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_DIAG_MASK, TMC2300_DIAG_SHIFT);
}

bool TMC2300Stepper::ioin_stepper() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_STEPPERCLK_INPUT_MASK,
                            TMC2300_STEPPERCLK_INPUT_SHIFT);
}

bool TMC2300Stepper::ioin_pdn_uart() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_PDN_UART_MASK, TMC2300_PDN_UART_SHIFT);
}

bool TMC2300Stepper::ioin_mode() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_MODE_INPUT_MASK, TMC2300_MODE_INPUT_SHIFT);
}

bool TMC2300Stepper::ioin_step() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_STEP_MASK, TMC2300_STEP_SHIFT);
}

bool TMC2300Stepper::ioin_dir() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_DIR_MASK, TMC2300_DIR_SHIFT);
}

bool TMC2300Stepper::ioin_comp_a1a2() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_COMP_A1A2_MASK, TMC2300_COMP_A1A2_SHIFT);
}

bool TMC2300Stepper::ioin_comp_b1b2() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_COMP_B1B2_MASK, TMC2300_COMP_B1B2_SHIFT);
}

int8_t TMC2300Stepper::ioin_version() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IOIN, TMC2300_VERSION_MASK, TMC2300_VERSION_SHIFT);
}

uint16_t TMC2300Stepper::gstat() { return tmc2300_readInt(&this->driver_, TMC2300_GSTAT); }
void TMC2300Stepper::gstat(uint16_t setting) { return tmc2300_writeInt(&this->driver_, TMC2300_GSTAT, setting); }

bool TMC2300Stepper::gstat_reset() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GSTAT, TMC2300_RESET_MASK, TMC2300_RESET_SHIFT);
}

void TMC2300Stepper::gstat_reset(bool clear) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GSTAT, TMC2300_RESET_MASK, TMC2300_RESET_SHIFT, clear);
}

bool TMC2300Stepper::gstat_drv_err() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GSTAT, TMC2300_DRV_ERR_MASK, TMC2300_DRV_ERR_SHIFT);
}

void TMC2300Stepper::gstat_drv_err(bool clear) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GSTAT, TMC2300_DRV_ERR_MASK, TMC2300_DRV_ERR_SHIFT, clear);
}

uint16_t TMC2300Stepper::gconf() { return tmc2300_readInt(&this->driver_, TMC2300_GCONF); }
void TMC2300Stepper::gconf(uint16_t setting) { return tmc2300_writeInt(&this->driver_, TMC2300_GCONF, setting); }

void TMC2300Stepper::gconf_en_spreadcycle(bool enable) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_EN_SPREADCYCLE_MASK, TMC2300_EN_SPREADCYCLE_SHIFT,
                       enable);
}

bool TMC2300Stepper::gconf_en_spreadcycle() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_EN_SPREADCYCLE_MASK, TMC2300_EN_SPREADCYCLE_SHIFT);
}

bool TMC2300Stepper::gconf_shaft() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_SHAFT_MASK, TMC2300_SHAFT_SHIFT);
}

void TMC2300Stepper::gconf_shaft(bool inverse) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_SHAFT_MASK, TMC2300_SHAFT_SHIFT, inverse);
}

void TMC2300Stepper::gconf_pdn_disable(bool disable) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_PDN_DISABLE_MASK, TMC2300_PDN_DISABLE_SHIFT, disable);
}

bool TMC2300Stepper::gconf_pdn_disable() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_PDN_DISABLE_MASK, TMC2300_PDN_DISABLE_SHIFT);
}

void TMC2300Stepper::gconf_mstep_reg_select(bool use) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_MSTEP_REG_SELECT_MASK, TMC2300_MSTEP_REG_SELECT_SHIFT,
                       use);
}

bool TMC2300Stepper::gconf_mstep_reg_select() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_MSTEP_REG_SELECT_MASK,
                            TMC2300_MSTEP_REG_SELECT_SHIFT);
}

void TMC2300Stepper::gconf_multistep_filt(bool enable) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_MULTISTEP_FILT_MASK, TMC2300_MULTISTEP_FILT_SHIFT,
                       enable);
}

bool TMC2300Stepper::gconf_multistep_filt() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_MULTISTEP_FILT_MASK, TMC2300_MULTISTEP_FILT_SHIFT);
}

void TMC2300Stepper::gconf_test_mode(bool enable) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_TEST_MODE_MASK, TMC2300_TEST_MODE_SHIFT, enable);
}

bool TMC2300Stepper::gconf_test_mode() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_TEST_MODE_MASK, TMC2300_TEST_MODE_SHIFT);
}

void TMC2300Stepper::gconf_diag_index(bool enable) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_DIAG_INDEX_MASK, TMC2300_DIAG_INDEX_SHIFT, enable);
}
bool TMC2300Stepper::gconf_diag_index() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_DIAG_INDEX_MASK, TMC2300_DIAG_INDEX_SHIFT);
}

void TMC2300Stepper::gconf_diag_step(bool enable) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_GCONF, TMC2300_DIAG_STEP_MASK, TMC2300_DIAG_STEP_SHIFT, enable);
}
bool TMC2300Stepper::gconf_diag_step() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_GCONF, TMC2300_DIAG_STEP_MASK, TMC2300_DIAG_STEP_SHIFT);
}

void TMC2300Stepper::coolstep_tcoolthrs(int32_t threshold) {
  tmc2300_writeInt(&this->driver_, TMC2300_TCOOLTHRS, threshold);
  this->coolstep_tcoolthrs_ = threshold;
}

int32_t TMC2300Stepper::coolstep_tcoolthrs() { return this->coolstep_tcoolthrs_; }

void TMC2300Stepper::chopconf_blank_time(uint8_t select) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_CHOPCONF, TMC2300_TBL_MASK, TMC2300_TBL_SHIFT, select);
}

void TMC2300Stepper::chopconf_mres(uint8_t index) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_CHOPCONF, TMC2300_MRES_MASK, TMC2300_MRES_SHIFT, index);
}

uint8_t TMC2300Stepper::chopconf_mres() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_CHOPCONF, TMC2300_MRES_MASK, TMC2300_MRES_SHIFT);
}

uint8_t TMC2300Stepper::ifcnt() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_IFCNT, TMC2300_IFCNT_MASK, TMC2300_IFCNT_SHIFT);
}

uint8_t TMC2300Stepper::slave_conf() {
  return TMC2300_FIELD_READ(&this->driver_, TMC2300_SLAVECONF, TMC2300_SLAVECONF_MASK, TMC2300_SLAVECONF_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
