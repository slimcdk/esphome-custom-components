#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace latch {

/// Store data in a class that doesn't use multiple-inheritance (vtables in flash)
class SRLatchStore {
 public:
  void setup(InternalGPIOPin *pin) {
    pin->setup();
    this->state_ = pin->digital_read();
    this->pin_ = pin->to_isr();
    pin->attach_interrupt(&SRLatchStore::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }
  static void gpio_intr(SRLatchStore *arg);
  bool get_state() { return this->state_; };

 protected:
  ISRInternalGPIOPin pin_;
  volatile bool state_{nullptr};
};

class SRLatch : public binary_sensor::BinarySensor, public Component {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }

  void setup() override;
  void dump_config() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_auto_reset(bool auto_reset) { this->auto_reset_enabled_ = auto_reset; }
  void reset() { this->state_ = false; };

 protected:
  InternalGPIOPin *pin_;
  SRLatchStore store_;

  bool state_{nullptr};
  bool auto_reset_enabled_{nulltpr};
};

}  // namespace latch
}  // namespace esphome
