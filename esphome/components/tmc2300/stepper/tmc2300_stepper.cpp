#include "tmc2300_stepper.h"
#include "esphome/components/tmc2300/tmc2300_api_registers.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2300_stepper {

void TMC2300Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2300 Stepper:");

#if defined(SERIAL_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: serial");
#elif defined(PULSES_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: pulses");
#else
  ESP_LOGE(TAG, "No control method defined!");
#endif

  LOG_TMC2300_PINS(this);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  LOG_TMC2300_VERSION(this);

  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps());
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz", this->clk_frequency_);
  ESP_LOGCONFIG(TAG, "  Velocity compensation: %f", this->vactual_factor_);
  const auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(OTTRIM_FIELD));
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot);

  LOG_STEPPER(this);

  LOG_TMC2300_CURRENTS(this);
  LOG_TMC2300_REGISTER_DUMP(this);
}

void TMC2300Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2300 Stepper...");
  TMC2300Component::setup();

  this->high_freq_.start();

  this->write_field(VACTUAL_FIELD, 0);

#if defined(PULSES_CONTROL)
  this->write_field(MULTISTEP_FILT_FIELD, false);
  this->write_field(DEDGE_FIELD, true);
#endif

#if defined(SERIAL_CONTROL)
  /* Configure INDEX for pulse feedback from the driver */
  // Check mux from figure 15.1 from datasheet rev1.09
  this->write_field(DEDGE_FIELD, false);
  this->write_field(INDEX_OTPW_FIELD, false);
  this->write_field(INDEX_STEP_FIELD, true);
  this->ips_.current_position_ptr = &this->current_position;
  this->ips_.direction_ptr = &this->current_direction;
  this->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_, gpio::INTERRUPT_ANY_EDGE);
#endif

  this->enable(true);

  ESP_LOGCONFIG(TAG, "TMC2300 Stepper setup done.");
}

void TMC2300Stepper::loop() {
  TMC2300Component::loop();

  // Compute speed and direction
  const time_t now = micros();
  this->calculate_speed_(now);
  const int32_t to_target = (this->target_position - this->current_position);
  this->current_direction = (to_target != 0 ? (Direction) (to_target / abs(to_target)) : Direction::STANDSTILL);

  int32_t vactual_ = this->speed_to_vactual(this->current_speed_);

#if defined(SERIAL_CONTROL)
  vactual_ *= this->current_direction;
  if (this->vactual_ != vactual_) {
    this->write_field(VACTUAL_FIELD, vactual_);
    this->vactual_ = vactual_;
  }
#endif

#if defined(PULSES_CONTROL)
  time_t dt = now - this->last_step_;
  if (dt >= (1 / (float) vactual_) * 1e6f) {
    if (this->direction_ != this->current_direction) {
      this->dir_pin_->digital_write(this->current_direction == Direction::BACKWARD);
      this->direction_ = this->current_direction;
    }
    this->step_pin_->digital_write(this->step_state_);
    this->step_state_ = !this->step_state_;
    this->current_position += (int32_t) this->current_direction;
    this->last_step_ = now;
  }
#endif
}

void TMC2300Stepper::set_target(int32_t steps) {
  if (!this->is_enabled_) {
    this->enable(true);
  }
  Stepper::set_target(steps);
}

void TMC2300Stepper::stop() {
  Stepper::stop();
#if defined(SERIAL_CONTROL)
  this->write_field(VACTUAL_FIELD, 0);
#endif
}

void TMC2300Stepper::enable(bool enable) {
  if (!enable) {
    this->stop();
  }
  TMC2300Component::enable(enable);
}

bool TMC2300Stepper::is_stalled() {
  if (this->current_direction == Direction::STANDSTILL) {
    return false;
  }

  const int32_t sgthrs = this->read_register(SGTHRS);
  const int32_t sgresult = this->read_register(SG_RESULT);
  return (sgthrs << 1) > sgresult;
}

}  // namespace tmc2300_stepper
}  // namespace esphome
