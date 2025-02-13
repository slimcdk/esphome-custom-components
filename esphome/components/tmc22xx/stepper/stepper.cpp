#include "stepper.h"
#include "esphome/components/tmc22xx/driver_registers.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc22xx {

void TMC22XXStepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC22XX Stepper:");
  LOG_STEPPER(this);
  LOG_TMC22XX(this);
}

void TMC22XXStepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC22XX Stepper...");
  TMC22XXComponent::setup();

  this->high_freq_.start();

  this->write_field(VACTUAL_FIELD, 0);

  if (this->control_method_ == ControlMethod::PULSES_CONTROL) {
    this->write_field(MULTISTEP_FILT_FIELD, false);
    this->write_field(DEDGE_FIELD, true);
  }

  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    /* Configure INDEX for pulse feedback from the driver */
    // Check mux from figure 15.1 from datasheet rev1.09
    this->write_field(DEDGE_FIELD, false);
    this->write_field(INDEX_OTPW_FIELD, false);
    this->write_field(INDEX_STEP_FIELD, true);
    this->ips_.current_position_ptr = &this->current_position;
    this->ips_.direction_ptr = &this->current_direction;
    this->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_, gpio::INTERRUPT_ANY_EDGE);
  }

  this->enable(true);

  ESP_LOGCONFIG(TAG, "TMC22XX Stepper setup done.");
}

void TMC22XXStepper::on_shutdown() { this->stop(); }

void TMC22XXStepper::loop() {
  TMC22XXComponent::loop();

  // Compute speed and direction
  const time_t now = micros();
  this->calculate_speed_(now);
  const int32_t to_target = (this->target_position - this->current_position);
  this->current_direction = (to_target != 0 ? (Direction) (to_target / abs(to_target)) : Direction::STANDSTILL);

  int32_t vactual_ = this->speed_to_vactual(this->current_speed_);

  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    vactual_ *= this->current_direction;
    if (this->vactual_ != vactual_) {
      this->write_field(VACTUAL_FIELD, vactual_);
      this->vactual_ = vactual_;
    }
  }

  if (this->control_method_ == ControlMethod::PULSES_CONTROL) {
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
  }
}

void TMC22XXStepper::set_target(int32_t steps) {
  if (this->control_method_ == ControlMethod::CONTROL_UNSET) {
    ESP_LOGE(TAG, "Control method not set!");
  }

  if (!this->is_enabled_) {
    this->enable(true);
  }
  Stepper::set_target(steps);
}

void TMC22XXStepper::stop() {
  Stepper::stop();
  if (this->control_method_ == ControlMethod::SERIAL_CONTROL) {
    this->write_field(VACTUAL_FIELD, 0);
  }
}

void TMC22XXStepper::enable(bool enable) {
  if (!enable) {
    this->stop();
  }
  TMC22XXComponent::enable(enable);
}

bool TMC22XXStepper::is_stalled() {
  if (this->current_direction == Direction::STANDSTILL) {
    return false;
  }

  const int32_t sgthrs = this->read_register(SGTHRS);
  const int32_t sgresult = this->read_register(SG_RESULT);
  return (sgthrs << 1) > sgresult;
}

}  // namespace tmc22xx
}  // namespace esphome
