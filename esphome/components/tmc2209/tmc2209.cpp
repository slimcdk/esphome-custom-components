
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

extern "C" {
void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209 *comp = components[channel];

  comp->write_array(data, writeLength);

  // chop off transmitted bytes from the buffer and flush due to one-wire uart filling up rx when transmitting
  comp->read_array(data, writeLength);
  comp->flush();

  if (readLength) {
    comp->read_array(data, readLength);
  }
}

uint8_t tmc2209_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }
}

void IRAM_ATTR HOT TMC2209IndexStore::gpio_intr(TMC2209IndexStore *arg) { arg->counter++; }
void IRAM_ATTR HOT TMC2209DiagStore::gpio_intr(TMC2209DiagStore *arg) {}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  LOG_PIN("  Index Pin: ", this->index_pin_);

  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", get_version());

  LOG_STEPPER(this);
}

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  this->channel_ = ++tmc2209_global_channel_index;
  components[this->channel_] = this;

  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(true);
    this->enable_pin_state_ = true;
  }

  // Initialize TMC driver instance
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2209_init(&this->driver_, this->channel_, this->address_, &this->config_, &tmc2209_defaultRegisterResetState[0]);
  tmc2209_reset(&this->driver_);

  // Need to disable PDN for UART read
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, 1);

  // Set toff
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TOFF_MASK, TMC2209_TOFF_SHIFT, 5);
  // Set blank time
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, 0);
  // Set hold current
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, 0);
  // Set run current
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, 16);
  // Set hold current decay delay
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, 15);
  // Set StealthChop
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_PWMCONF, TMC2209_PWM_AUTOSCALE_MASK, TMC2209_PWM_AUTOSCALE_SHIFT, 0);
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_PWMCONF, TMC2209_PWM_GRAD_MASK, TMC2209_PWM_GRAD_SHIFT, 0);

  // Inttrupt handling for index events
  this->index_pin_->setup();
  this->index_store_.index_pin = this->index_pin_->to_isr();
  this->index_pin_->attach_interrupt(TMC2209IndexStore::gpio_intr, &this->index_store_, gpio::INTERRUPT_RISING_EDGE);

  // Inttrupt handling for diag events
  this->diag_pin_->setup();
  this->diag_store_.diag_pin = this->diag_pin_->to_isr();
  this->diag_pin_->attach_interrupt(TMC2209DiagStore::gpio_intr, &this->diag_store_, gpio::INTERRUPT_ANY_EDGE);

  this->disable();

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void TMC2209::loop() {
  tmc2209_periodicJob(&this->driver_, 0);  // update the registers
}

// Enable/disable driver
void TMC2209::enable(bool enable) {
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(enable);
    this->enable_pin_state_ = enable;
  }
}

void TMC2209::enable() { this->enable(true); }
void TMC2209::disable() { this->enable(false); }

bool TMC2209::get_ioin_enn() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_ENN_MASK, TMC2209_ENN_SHIFT);
}
bool TMC2209::get_ioin_ms1() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_MS1_MASK, TMC2209_MS2_SHIFT);
}
bool TMC2209::get_ioin_ms2() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_MS2_MASK, TMC2209_MS2_SHIFT);
}
bool TMC2209::get_ioin_diag() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIAG_MASK, TMC2209_DIAG_SHIFT);
}
bool TMC2209::get_ioin_pdn_uart() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_PDN_UART_MASK, TMC2209_PDN_UART_SHIFT);
}
bool TMC2209::get_ioin_step() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_STEP_MASK, TMC2209_STEP_SHIFT);
}
bool TMC2209::get_ioin_spread_en() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_SEL_A_MASK, TMC2209_SEL_A_SHIFT);
}
bool TMC2209::get_ioin_dir() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIR_MASK, TMC2209_DIR_SHIFT);
}

int32_t TMC2209::get_version() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
}

void TMC2209::set_velocity(int32_t velocity) {
  this->enable((bool) velocity);
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, velocity);
}
void TMC2209::set_inverse_direction(bool inverse_direction) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, inverse_direction);
}
void TMC2209::set_microsteps(uint8_t ms) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, ms);
}
void TMC2209::set_run_current(int32_t current) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, current);
}
void TMC2209::set_hold_current(int32_t current) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, current);
}
void TMC2209::set_hold_current_delay(int32_t delay) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, delay);
}
void TMC2209::set_tcool_threshold(int32_t threshold) {
  // TMC2209_FIELD_WRITE(&this->driver_, TMC2209_TCOOLTHRS, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, threshold);
}
void TMC2209::set_sg_stall_threshold(int32_t threshold) {
  // TMC2209_FIELD_WRITE(&this->driver_, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, threshold);
}

void TMC2209::stop_motion() { this->set_velocity(0); }

void TMC2209::set_target(int32_t steps) {
  this->target_position = steps;
  // this->index_store_
}

}  // namespace tmc
}  // namespace esphome
