#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace gpio {

#ifdef USE_INTERRUPT
/// Store data in a class that doesn't use multiple-inheritance (vtables in flash)
class GPIOBinarySensorStore {
 public:
  void setup(InternalGPIOPin *pin) {
    pin->setup();
    this->state_ = pin->digital_read();
    this->pin_ = pin->to_isr();
    pin->attach_interrupt(&GPIOBinarySensorStore::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }
  static void gpio_intr(GPIOBinarySensorStore *arg);
  bool get_state() { return this->state_; };

 protected:
  ISRInternalGPIOPin pin_;
  volatile bool state_{nullptr};
};
#endif

class GPIOBinarySensor : public binary_sensor::BinarySensor, public Component {
 public:
#ifdef USE_INTERRUPT
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
#else
  void set_pin(GPIOPin *pin) { pin_ = pin; }
#endif

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Setup pin
  void setup() override;
  void dump_config() override;
  /// Hardware priority
  float get_setup_priority() const override;
  /// Check sensor
  void loop() override;

 protected:
#ifdef USE_INTERRUPT
  InternalGPIOPin *pin_;
  GPIOBinarySensorStore store_;
#else
  GPIOPin *pin_;
#endif
  bool state_{nullptr};
};

}  // namespace gpio
}  // namespace esphome
