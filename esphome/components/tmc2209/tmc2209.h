#pragma once

#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/uart/uart.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

#define MAX_ALLOWED_COMPONENTS 3

namespace esphome {
namespace tmc {

class TMC2209 : public stepper::Stepper, public Component, public uart::UARTDevice {
 public:
  TMC2209(uint8_t address, InternalGPIOPin *index_pin, InternalGPIOPin *diag_pin)
      : address_(address), index_pin_(index_pin), diag_pin_(diag_pin) {}

  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  int32_t get_version();

  uint8_t get_address() { return this->address_; };
  TMC2209TypeDef *get_driver() { return &this->driver_; };

 protected:
  // TMC API stuff
  uint8_t channel_ = 0;
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  uint8_t address_;

  bool enable_pin_state_;

  GPIOPin *enable_pin_;
  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;
};

static TMC2209 *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_global_channel_index = 0;

template<typename... Ts> class TMC2209ConfigureAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(int, velocity)
  TEMPLATABLE_VALUE(int /*float*/, run_current)
  TEMPLATABLE_VALUE(int /*float*/, hold_current)
  TEMPLATABLE_VALUE(int /*float*/, hold_current_delay)
  TEMPLATABLE_VALUE(int, tcool_threshold)
  TEMPLATABLE_VALUE(int, stall_threshold)

  void play(Ts... x) override {
    TMC2209TypeDef *driver = this->parent_->get_driver();

    // set inverse direction
    if (this->inverse_direction_.has_value()) {
      auto inverse_direction_val = this->inverse_direction_.value(x...);
      ESP_LOGW("tmc2209", "inverse direction %d", inverse_direction_val);
      TMC2209_FIELD_WRITE(driver, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, inverse_direction_val);
    }

    // Set microstepping
    if (this->microsteps_.has_value()) {
      auto microsteps_val = this->microsteps_.value(x...);
      ESP_LOGW("tmc2209", "microsteps %d", microsteps_val);
      TMC2209_FIELD_WRITE(driver, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, microsteps_val);
    }

    // Set velocity
    if (this->velocity_.has_value()) {
      auto velocity_val = this->velocity_.value(x...);
      ESP_LOGW("tmc2209", "velocity %d", velocity_val);
      TMC2209_FIELD_WRITE(driver, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, velocity_val);
    }

    // Set run current
    if (this->run_current_.has_value()) {
      auto run_current_val = this->run_current_.value(x...);
      ESP_LOGW("tmc2209", "run current %d", run_current_val);
      TMC2209_FIELD_WRITE(driver, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, run_current_val);
    }

    // Set hold current
    if (this->hold_current_.has_value()) {
      auto hold_current_val = this->hold_current_.value(x...);
      ESP_LOGW("tmc2209", "hold current %d", hold_current_val);
      TMC2209_FIELD_WRITE(driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, hold_current_val);
    }

    // Set hold current decay delay
    if (this->hold_current_delay_.has_value()) {
      auto hold_current_delay_val = this->hold_current_delay_.value(x...);
      ESP_LOGW("tmc2209", "hold current delay %d", hold_current_delay_val);
      TMC2209_FIELD_WRITE(driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT,
                          hold_current_delay_val);
    }

    // Set tcoll threshold
    if (this->tcool_threshold_.has_value()) {
      auto tcool_threshold_val = this->tcool_threshold_.value(x...);
      ESP_LOGW("tmc2209", "tcool_threshold %d", tcool_threshold_val);
      // TMC2209_FIELD_WRITE(driver, TMC2209_TCOOLTHRS, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT,
      // tcool_threshold_val);
    }

    // set stallguard threshold
    if (this->stall_threshold_.has_value()) {
      auto stall_threshold_val = this->stall_threshold_.value(x...);
      ESP_LOGW("tmc2209", "stall_threshold %d", stall_threshold_val);
      // TMC2209_FIELD_WRITE(driver, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, stall_threshold_val);
    }
  }
};

}  // namespace tmc
}  // namespace esphome
