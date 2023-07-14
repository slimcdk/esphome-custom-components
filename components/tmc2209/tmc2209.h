#pragma once

#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/uart/uart.h"

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

/*
template<typename... Ts> class TMC2209SetupAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(int, tcool_threshold)
  TEMPLATABLE_VALUE(int, stall_threshold)
  TEMPLATABLE_VALUE(float, current)

  void play(Ts... x) override {
    auto driver = this->parent_->get_driver();
    if (this->microsteps_.has_value()) {
      ESP_LOGW("tmc2209", "microsteps %d", this->microsteps_.value(x...));
      driver.microsteps(this->microsteps_.value(x...));
    }
    if (this->tcool_threshold_.has_value()) {
      ESP_LOGW("tmc2209", "tcool_threshold %d", this->tcool_threshold_.value(x...));
      driver.TCOOLTHRS(this->tcool_threshold_.value(x...));
    }
    if (this->stall_threshold_.has_value()) {
      ESP_LOGW("tmc2209", "stall %d", this->stall_threshold_.value(x...));
      driver.SGTHRS(this->stall_threshold_.value(x...));
    }
    if (this->current_.has_value()) {
      ESP_LOGW("tmc2209", "current %.3f", this->current_.value(x...));
      driver.rms_current(static_cast<int>(this->current_.value(x...) * 1000.0));
    }
  }
};
*/

}  // namespace tmc
}  // namespace esphome
