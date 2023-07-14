#pragma once

#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/uart/uart.h"

#include "register_map.h"

namespace esphome {
namespace tmc {

/// Store data in a class that doesn't use multiple-inheritance (vtables in flash)
class TMCDiagStore {
 public:
  void setup(InternalGPIOPin *pin) {
    pin->setup();
    this->pin_ = pin->to_isr();
    pin->attach_interrupt(&TMCDiagStore::gpio_intr, this, gpio::INTERRUPT_RISING_EDGE);
  }
  static void gpio_intr(TMCDiagStore *arg);
  bool active() { return this->triggered; };
  void reset() { this->triggered = false; };

 protected:
  ISRInternalGPIOPin pin_;
  volatile bool triggered{false};
};

class TMC2209 : public stepper::Stepper, public Component, public uart::UARTDevice {
 public:
  TMC2209() = default;
  TMC2209(const TMC2209 &) = delete;
  TMC2209 &operator=(const TMC2209 &) = delete;
  ~TMC2209();

  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; }
  void set_direction_pin(GPIOPin *pin) { this->dir_pin_ = pin; }
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  GPIOPin *step_pin_;
  GPIOPin *dir_pin_;
  GPIOPin *enable_pin_{nullptr};
  InternalGPIOPin *diag_pin_{nullptr};

  bool enable_pin_state_;
  HighFrequencyLoopRequester high_freq_;

  // TMC diag
  TMCDiagStore diag_;
};

}  // namespace tmc
}  // namespace esphome
