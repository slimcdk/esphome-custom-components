#pragma once

#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/uart/uart.h"

#include "register_map.h"

namespace esphome {
namespace tmc {

class TMC2209 : public stepper::Stepper, public Component, public uart::UARTDevice {
 public:
  TMC2209() = default;
  TMC2209(const TMC2209 &) = delete;
  TMC2209 &operator=(const TMC2209 &) = delete;
  ~TMC2209();

  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; }
  void set_direction_pin(GPIOPin *pin) { this->dir_pin_ = pin; }
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  GPIOPin *step_pin_;
  GPIOPin *dir_pin_;
  GPIOPin *enable_pin_{nullptr};

  bool enable_pin_state_;
  HighFrequencyLoopRequester high_freq_;
};

}  // namespace tmc
}  // namespace esphome
