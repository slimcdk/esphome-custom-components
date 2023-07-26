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

class TMC2209;

static TMC2209 *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_global_channel_index = 0;

struct TMC2209IndexStore {
  ISRInternalGPIOPin index_pin;

  volatile int32_t counter{0};
  int32_t target{0};

  static void gpio_intr(TMC2209IndexStore *arg);
};

struct TMC2209DiagStore {
  ISRInternalGPIOPin diag_pin;
  volatile bool triggered;
  void set_flag(bool high);

  static void gpio_intr(TMC2209DiagStore *arg);
};

class TMC2209 : public stepper::Stepper, public Component, public uart::UARTDevice {
 public:
  TMC2209(uint8_t address, InternalGPIOPin *index_pin, InternalGPIOPin *diag_pin)
      : address_(address), index_pin_(index_pin), diag_pin_(diag_pin) {}

  /* Method overrides */
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps) override;

  void enable(bool enable);
  void enable();
  void disable();
  void stop_motion();

  /* Setters */
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_inverse_direction(bool inverse);
  void set_velocity(int32_t velocity);
  void set_microsteps(uint8_t ms);
  void set_run_current(int32_t current);
  void set_hold_current(int32_t current);
  void set_hold_current_delay(int32_t current);
  void set_tcool_threshold(int32_t threshold);
  void set_sg_stall_threshold(int32_t threshold);

  /* Getters */
  uint8_t get_address() { return this->address_; };
  bool get_ioin_enn();
  bool get_ioin_ms1();
  bool get_ioin_ms2();
  bool get_ioin_diag();
  bool get_ioin_pdn_uart();
  bool get_ioin_step();
  bool get_ioin_spread_en();
  bool get_ioin_dir();
  int32_t get_version();

  TMC2209TypeDef *get_driver() { return &this->driver_; };

 protected:
  /* TMC API stuff */
  uint8_t channel_ = 0;
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  uint8_t address_;
  /* */

  bool enable_pin_state_;

  GPIOPin *enable_pin_;
  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;

  TMC2209IndexStore index_store_{};
  TMC2209DiagStore diag_store_{};
};

template<typename... Ts> class TMC2209ConfigureAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(int, velocity)
  TEMPLATABLE_VALUE(int /*float*/, run_current)
  TEMPLATABLE_VALUE(int /*float*/, hold_current)
  TEMPLATABLE_VALUE(int /*float*/, hold_current_delay)
  TEMPLATABLE_VALUE(int, tcool_threshold)
  TEMPLATABLE_VALUE(int, sg_stall_threshold)

  void play(Ts... x) override {
    TMC2209TypeDef *driver = this->parent_->get_driver();

    // set inverse direction
    if (this->inverse_direction_.has_value())
      this->parent_->set_inverse_direction(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_velocity(this->microsteps_.value(x...));

    if (this->velocity_.has_value())
      this->parent_->set_velocity(this->velocity_.value(x...));

    if (this->run_current_.has_value())
      this->parent_->set_run_current(this->run_current_.value(x...));

    if (this->hold_current_.has_value())
      this->parent_->set_hold_current(this->hold_current_.value(x...));

    if (this->hold_current_delay_.has_value())
      this->parent_->set_hold_current_delay(this->hold_current_delay_.value(x...));

    if (this->tcool_threshold_.has_value())
      this->parent_->set_tcool_threshold(this->tcool_threshold_.value(x...));

    if (this->sg_stall_threshold_.has_value())
      this->parent_->set_sg_stall_threshold(this->sg_stall_threshold_.value(x...));
  }
};

}  // namespace tmc
}  // namespace esphome
