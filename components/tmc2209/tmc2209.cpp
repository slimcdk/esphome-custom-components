#include "tmc2209.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");
  LOG_PIN("  Step Pin: ", this->step_pin_);
  LOG_PIN("  Dir Pin: ", this->dir_pin_);
  LOG_PIN("  Sleep Pin: ", this->enable_pin_);
  LOG_STEPPER(this);
}

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  this->write_str("Hello from UART");
}

void TMC2209::loop() {
  bool at_target = this->has_reached_target();
  if (this->enable_pin_ != nullptr) {
    bool sleep_rising_edge = !enable_pin_state_ & !at_target;
    this->enable_pin_->digital_write(!at_target);
    this->enable_pin_state_ = !at_target;
    if (sleep_rising_edge) {
      delayMicroseconds(1000);
    }
  }
  if (at_target) {
    this->high_freq_.stop();
  } else {
    this->high_freq_.start();
  }

  int32_t dir = this->should_step_();
  if (dir == 0)
    return;

  this->dir_pin_->digital_write(dir == 1);
  this->step_pin_->digital_write(true);
  delayMicroseconds(5);
  this->step_pin_->digital_write(false);
}

}  // namespace tmc
}  // namespace esphome
