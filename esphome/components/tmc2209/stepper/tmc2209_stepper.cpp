#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "tmc2209_stepper.h"

namespace esphome {
namespace tmc2209 {

static const char *TAG = "tmc2209.stepper";

void TMC2209Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Stepper...");

  this->enn_pin_->setup();

  /* Reconfigure INDEX as it serves another purpose here. */
  // Check mux from figure 15.1 from datasheet rev1.09
  this->parent_->gconf_index_step(true);
  this->parent_->gconf_index_otpw(false);

  this->ips_.target_position_ptr = &this->target_position;
  this->ips_.current_position_ptr = &this->current_position;
  this->ips_.direction_ptr = &this->direction_;

  // Override interrupt set by parent on index_pin.
  this->parent_->index_pin_->detach_interrupt();
  this->parent_->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_, gpio::INTERRUPT_RISING_EDGE);
  /* Reconfiguration done */

  // this->set_interval(1000, [this]() { ESP_LOGD(TAG, "current position %d", this->current_position); });

  this->prev_has_reached_target_ = this->has_reached_target();
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");
  LOG_PIN("  ENN pin: ", this->enn_pin_);
  LOG_STEPPER(this);
}

void TMC2209Stepper::run_driver_activation_() {
  // Control enable/disable standstill hold taken into consideration
  const bool has_reached_target_ = this->has_reached_target();
  if (this->prev_has_reached_target_ != has_reached_target_) {
    // Only trigger start/stop moving of stepper on changes, not every loop iteration

    if (!has_reached_target_) {
      // Start moving

      this->enable(true);
      this->cancel_timeout(DRIVER_STATE_TIMER_NAME);
      this->high_freq_.start();

      // Attempt to detect faulty index feedback
      this->set_timeout(INDEX_FB_CHECK_TIMER_NAME, 1000, [this, from = this->current_position]() {
        if (this->current_position == from) {
          ESP_LOGE(TAG, "No stepping feedback received. Is your index-pin wired correctly?");
          this->stop();
          this->mark_failed();
        } else {
          ESP_LOGVV(TAG, "INDEX feedback received.");
        }
      });

    } else {
      // Stopped moving. Delayed disable is used to let the motor settle before actually turning it off
      this->set_timeout(DRIVER_STATE_TIMER_NAME, 100, [this]() { this->enable(false); });
      this->cancel_timeout(INDEX_FB_CHECK_TIMER_NAME);
      this->high_freq_.stop();
    }
  }
  this->prev_has_reached_target_ = has_reached_target_;
}

void TMC2209Stepper::loop() {
  this->run_driver_activation_();

  // Compute and set speed and direction to move the motor in
  const int32_t to_target = (this->target_position - this->current_position);
  this->direction_ = (to_target != 0 ? (Direction) (to_target / abs(to_target)) : Direction::NONE);  // yield 1, -1 or 0
  this->calculate_speed_(micros());
  // -2.8 inverts direction and scales stepping to match specified. Magic number
  this->parent_->vactual((-2.8 * this->direction_) * (int32_t) this->current_speed_);
}

void TMC2209Stepper::stop() {
  this->direction_ = Direction::NONE;
  this->target_position = this->current_position;
  this->parent_->vactual(0);
}

void TMC2209Stepper::enable(bool enable) {
  this->driver_is_enabled_ = enable;
  this->enn_pin_->digital_write(!enable);
}

}  // namespace tmc2209
}  // namespace esphome
