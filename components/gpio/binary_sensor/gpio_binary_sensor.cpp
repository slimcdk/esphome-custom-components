#include "gpio_binary_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gpio {

static const char *const TAG = "gpio.binary_sensor";

#ifdef USE_INTERRUPT
void IRAM_ATTR GPIOBinarySensorStore::gpio_intr(GPIOBinarySensorStore *arg) { arg->state_ = arg->pin_.digital_read(); }
#endif

void GPIOBinarySensor::setup() {
#ifdef USE_INTERRUPT
  this->store_.setup(this->pin_);
  this->state_ = this->store_.get_state();
#else
  this->pin_->setup();
  this->state_ = this->pin_->digital_read();
#endif
  this->publish_initial_state(this->state_);
}

void GPIOBinarySensor::loop() {
#ifdef USE_INTERRUPT
  this->state_ = this->store_.get_state();
#else
  this->state_ = this->pin_->digital_read();
#endif
  this->publish_state(this->state_);
}

void GPIOBinarySensor::dump_config() {
  LOG_BINARY_SENSOR("", "GPIO Binary Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
#ifdef USE_INTERRUPT
  ESP_LOGCONFIG(TAG, "  Configured with interrupt");
#endif
}

float GPIOBinarySensor::get_setup_priority() const { return setup_priority::HARDWARE; }

}  // namespace gpio
}  // namespace esphome
