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

// static TMC2209 *comp = nullptr;

class TMC2209 : public stepper::Stepper, public Component, public uart::UARTDevice {
 public:
  TMC2209() = default;
  TMC2209(const TMC2209 &) = delete;
  TMC2209 &operator=(const TMC2209 &) = delete;
  ~TMC2209();

  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  TMC2209TypeDef *get_driver() { return &this->driver; };

 protected:
  GPIOPin *enable_pin_;
  bool enable_pin_state_;

  uint32_t last_run_ = 0;

  // TMC API stuff
  uint8_t channel_ = 0;
  uint8_t address_ = 0x0;
  TMC2209TypeDef driver;
  ConfigurationTypeDef config;
};

static TMC2209 *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_global_channel_index = 0;

template<typename... Ts> class TMC2209SetupAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, direction)
  TEMPLATABLE_VALUE(int, velocity)

  void play(Ts... x) override {
    TMC2209TypeDef *driver = this->parent_->get_driver();

    if (this->direction_.has_value()) {
      ESP_LOGW("tmc2209", "direction %d", this->direction_.value(x...));
      TMC2209_FIELD_WRITE(driver, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, this->direction_.value(x...));
    }

    if (this->velocity_.has_value()) {
      ESP_LOGW("tmc2209", "velocity %d", this->velocity_.value(x...));
      TMC2209_FIELD_WRITE(driver, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT,
                          this->velocity_.value(x...));
    }
  }
};

}  // namespace tmc
}  // namespace esphome
