#include "gpio_binary_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace latch {

static const char *const TAG = "gpio.binary_sensor";

void IRAM_ATTR SRLatchStore::gpio_intr(SRLatchStore *arg) { arg->state_ = arg->pin_.digital_read(); }

void SRLatch::setup() {
  this->store_.setup(this->pin_);
  this->state_ = this->store_.get_state();
  this->publish_initial_state(this->state_);
}

void SRLatch::loop() {
  if (!this->state_) {
    this->state_ = this->store_.get_state();

    if (this->auto_reset_enabled_) {
      this->reset();
    }
  }

  this->publish_state(this->state_);
}

void SRLatch::dump_config() {
  LOG_BINARY_SENSOR("", "GPIO Latch Binary Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->auto_reset_enabled_) {
    ESP_LOGCONFIG(TAG, "  Auto reset enabled");
  } else {
    ESP_LOGCONFIG(TAG, "  Auto reset dissabled");
  }
}

float SRLatch::get_setup_priority() const { return setup_priority::HARDWARE; }

}  // namespace latch
}  // namespace esphome
