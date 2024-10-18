#include "tmc2209_stepper.h"
#include "esphome/components/tmc2209/tmc2209_api_registers.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");

#if defined(USE_UART_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: Over UART with feedback on INDEX");
#elif defined(USE_PULSE_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: Pulses on STEP and direction on DIR");
#else
  ESP_LOGE(TAG, "No control method defined!");
#endif

  LOG_TMC2209_PINS(this);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  LOG_TMC2209_VERSION(this);

  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps());
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz", this->clk_frequency_);
  const auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(OTTRIM_FIELD));
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot);

  const uint sdal0_ = (this->max_speed_ * this->sdal_);
  const float sdal1_ = (this->sdal_ * 100.0);
  ESP_LOGCONFIG(TAG, "  Activate stall detection: At %d steps/s (%.0f%% of max speed)", sdal0_, sdal1_);

  LOG_STEPPER(this);

  LOG_TMC2209_CURRENTS(this);
  LOG_TMC2209_REGISTER_DUMP(this);
}

void TMC2209Stepper::setup() {
  TMC2209Component::setup();
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Stepper...");

  this->write_field(VACTUAL_FIELD, 0);

#if defined(USE_UART_CONTROL)
  /* Configure INDEX for pulse feedback from the driver */
  // Check mux from figure 15.1 from datasheet rev1.09
  this->write_field(INDEX_STEP_FIELD, true);
  this->write_field(INDEX_OTPW_FIELD, false);
  this->ips_.current_position_ptr = &this->current_position;
  this->ips_.direction_ptr = &this->current_direction_;
  // this->index_pin_->setup();
  this->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_, gpio::INTERRUPT_RISING_EDGE);
#endif

#if defined(USE_PULSE_CONTROL)
  // TODO: Test if below is beneficial to have
  // this->write_field(MULTISTEP_FILT_FIELD, false);
  // this->write_field(DEDGE_FIELD, true);
#endif

#if defined(USE_HYBRID_CONTROL)

#endif

  this->enable(true);

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::loop() {
  TMC2209Component::loop();

  const bool standstill = this->read_field(STST_FIELD);
  if (!standstill && this->current_speed_ >= (this->max_speed_ * this->sdal_)) {
    this->stalled_handler_.check(this->is_stalled());
  }

  /** Serial control handling **/
#if defined(USE_UART_CONTROL)
  // Compute and set speed and direction to move the motor in
  const int32_t to_target = (this->target_position - this->current_position);
  this->current_direction_ = (to_target != 0 ? (stepper::Direction)(to_target / abs(to_target))
                                             : stepper::Direction::STANDSTILL);  // yield 1, -1 or 0

  this->calculate_speed_(micros());
  // -2.8 magically to synchronizes feedback with set velocity
  const uint32_t velocity = ((int8_t) this->current_direction_) * this->current_speed_ * 2.8;
  if (this->read_field(VACTUAL_FIELD) != velocity) {
    this->write_field(VACTUAL_FIELD, velocity);
  }
#endif
  /** */

/** Pulse train handling **/
#if defined(USE_PULSE_CONTROL)
  // Compute and set speed and direction to move the motor in
  this->current_direction_ = this->should_step_();
  if (this->current_direction_ != stepper::Direction::STANDSTILL) {
    this->dir_pin_->digital_write(this->current_direction_ == stepper::Direction::FORWARD);
    delayMicroseconds(5);
    this->step_pin_->digital_write(true);
    delayMicroseconds(5);
    this->step_pin_->digital_write(false);
  }
#endif
/** */

/** Pulse train and serial handling **/
#if defined(USE_HYBRID_CONTROL)

#endif
  /** */
}

void TMC2209Stepper::set_target(int32_t steps) {
  if (!this->is_enabled_) {
    this->enable(true);
  }

  stepper::Stepper::set_target(steps);
}

void TMC2209Stepper::stop() {
  stepper::Stepper::stop();

#if defined(USE_UART_CONTROL)
  this->write_field(VACTUAL_FIELD, 0);
#endif
}

void TMC2209Stepper::enable(bool enable) {
  if (!enable) {
    this->stop();
  }

  if (this->enn_pin_ != nullptr) {
    this->enn_pin_->digital_write(!enable);
  } else {
    this->write_field(TOFF_FIELD, enable ? 3 : 0);
  }
  this->is_enabled_ = enable;
}

}  // namespace tmc2209
}  // namespace esphome
