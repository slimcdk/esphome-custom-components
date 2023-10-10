

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc2300.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2300.stepper";

void IRAM_ATTR HOT TMC2300ISRStore::pulse_isr(TMC2300ISRStore *arg) {
  // TODO check if pointers are set

  (*(arg->current_position_ptr)) += *(arg->direction_ptr);
  if (*(arg->current_position_ptr) == *(arg->target_position_ptr)) {
    arg->enn_pin_.digital_write(false);
    *(arg->enn_pin_state_ptr) = false;
    *(arg->direction_ptr) = Direction::NONE;
    *(arg->target_position_ptr) = *(arg->current_position_ptr);
  }
}

void IRAM_ATTR HOT TMC2300ISRStore::fault_isr(TMC2300ISRStore *arg) { (*(arg->fault_detected_ptr)) = true; }

// Global list of driver to work around TMC channel index
TMC2300Stepper::TMC2300Stepper() {
  this->index_ = tmc2300_stepper_global_index++;
  components[this->index_] = this;

  // Initialize TMC-API object
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2300_init(&this->driver_, this->index_, &this->config_, &tmc2300_defaultRegisterResetState[0]);
}

void TMC2300Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2300...");

  this->enn_pin_->setup();
  this->enn_pin_->digital_write(this->enn_pin_state_);

  this->index_pin_->setup();
  this->index_pin_->attach_interrupt(TMC2300ISRStore::pulse_isr, &this->isr_store_, gpio::INTERRUPT_FALLING_EDGE);

  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(TMC2300ISRStore::fault_isr, &this->isr_store_, gpio::INTERRUPT_RISING_EDGE);

  // Pulse tracker setup
  this->isr_store_.enn_pin_ = this->enn_pin_->to_isr();
  this->isr_store_.target_position_ptr = &this->target_position;
  this->isr_store_.current_position_ptr = &this->current_position;
  this->isr_store_.direction_ptr = &this->direction_;
  this->isr_store_.enn_pin_state_ptr = &this->enn_pin_state_;
  this->isr_store_.fault_detected_ptr = &this->fault_detected_;

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

  ESP_LOGCONFIG(TAG, "TMC2300 Stepper setup done.");
}

void TMC2300Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2300 Stepper:");
  LOG_PIN("  Enn Pin: ", this->enn_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  LOG_PIN("  Index Pin: ", this->index_pin_);
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", this->ioin_version());
  LOG_STEPPER(this);
}

void TMC2300Stepper::loop() {
  this->update_registers_();

  if (this->fault_detected_) {
    this->fault_detected_ = false;
    this->on_fault_signal_callback_.call();
    // this->stop();
  }

  if (this->has_reached_target()) {
    this->stop();
    return;
  }

  this->calculate_speed_(micros());
  this->vactual(this->current_speed_ * this->direction_);
}

void TMC2300Stepper::set_target(int32_t target) {
  Stepper::set_target(target);

  const int32_t steps_to_target = (this->target_position - this->current_position);
  if (steps_to_target == 0) {
    return;
  }

  this->direction_ = (Direction) (steps_to_target / abs(steps_to_target));  // yield 1 or -1
  this->enable(true);
}

void TMC2300Stepper::stop() {
  this->direction_ = Direction::NONE;
  this->target_position = this->current_position;
  this->vactual(0);
  this->enable(false);
}

void TMC2300Stepper::enable(bool enable) {
  this->enn_pin_->digital_write(enable);
  this->enn_pin_state_ = enable;
}

bool TMC2300Stepper::reset_() { return tmc2300_reset(&this->driver_); }
bool TMC2300Stepper::restore_() { return tmc2300_restore(&this->driver_); }
void TMC2300Stepper::update_registers_() { return tmc2300_periodicJob(&this->driver_, 0); }

uint16_t TMC2300Stepper::get_microsteps() {
  const uint8_t index = this->chopconf_mres();
  switch (index) {
    case 0:
      return 256;
    case 1:
      return 128;
    case 2:
      return 64;
    case 3:
      return 32;
    case 4:
      return 16;
    case 5:
      return 8;
    case 6:
      return 4;
    case 7:
      return 2;
    case 8:
      return 1;
    default:
      return 0;
  }
}

void TMC2300Stepper::set_microsteps(uint16_t ms) {
  switch (ms) {
    case 1:
      return this->chopconf_mres(8);
    case 2:
      return this->chopconf_mres(7);
    case 4:
      return this->chopconf_mres(6);
    case 8:
      return this->chopconf_mres(5);
    case 16:
      return this->chopconf_mres(4);
    case 32:
      return this->chopconf_mres(3);
    case 64:
      return this->chopconf_mres(2);
    case 128:
      return this->chopconf_mres(1);
    case 256:
      return this->chopconf_mres(0);
    default:
      return this->chopconf_mres(0);
  }
  return;
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

void TMC2300Stepper::ihold_irun_ihold(int32_t current) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IRUN_MASK, TMC2300_IRUN_SHIFT, current);
}

void TMC2300Stepper::ihold_irun_irun(int32_t current) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IHOLD_MASK, TMC2300_IHOLD_SHIFT, current);
}

void TMC2300Stepper::ihold_irun_ihold_delay(int32_t delay) {
  TMC2300_FIELD_UPDATE(&this->driver_, TMC2300_IHOLD_IRUN, TMC2300_IHOLDDELAY_MASK, TMC2300_IHOLDDELAY_SHIFT, delay);
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
