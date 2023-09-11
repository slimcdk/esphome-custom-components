#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"

#include "esphome/components/stepper/stepper.h"
#include "esphome/components/tmc2209/tmc2209.h"

namespace esphome {
namespace tmc {

struct TMC2209StepperIndexStore {
  ISRInternalGPIOPin index_pin;

  volatile int32_t current_{0};
  int32_t target_{0};
  volatile bool target_reached_{true};

  static void gpio_intr(TMC2209StepperIndexStore *arg);
};

class TMC2209Stepper : public Component, public stepper::Stepper, public TMC2209 {
 public:
  TMC2209Stepper() = default;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; }

  void set_target(int32_t steps) override;
  void report_position(int32_t steps) override;

  void stop();

  void add_on_motor_stall_callback(std::function<void()> callback) {
    this->on_motor_stall_callback_.add(std::move(callback));
  }

 protected:
  TMC2209StepperIndexStore index_store_{};

  CallbackManager<void()> on_motor_stall_callback_;

  bool stop_on_fault_;

  uint32_t prev_time_;
  uint32_t prev_position_{0};
  uint32_t sg_thrs_;
};

class TMC2209StepperMotorStallTrigger : public Trigger<> {
 public:
  explicit TMC2209StepperMotorStallTrigger(TMC2209Stepper *parent) {
    parent->add_on_motor_stall_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc
}  // namespace esphome
