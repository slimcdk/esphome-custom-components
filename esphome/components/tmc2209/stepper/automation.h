#pragma once
#include "tmc2209_stepper.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace tmc2209 {

template<typename... Ts> class ActivationAction : public Action<Ts...>, public Parented<TMC2209Stepper> {
  TEMPLATABLE_VALUE(bool, activate)

  void play(Ts... x) override {
    if (this->activate_.has_value()) {
      this->parent_->enable(this->activate_.value(x...));
    }
  }
};

template<typename... Ts> class StallguardAction : public Action<Ts...>, public Parented<TMC2209Stepper> {
 public:
  TEMPLATABLE_VALUE(int32_t, stallguard_threshold)
  TEMPLATABLE_VALUE(float, activation_level)

  void play(Ts... x) override {
    if (this->stallguard_threshold_.has_value())
      this->parent_->write_register(SGTHRS, this->stallguard_threshold_.value(x...));

    if (this->activation_level_.has_value())
      this->parent_->set_stall_detection_activation_level(this->activation_level_.value(x...));
  }
};

class OnStallTrigger : public Trigger<> {
 public:
  explicit OnStallTrigger(TMC2209Stepper *parent) {
    parent->add_on_stall_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc2209
}  // namespace esphome
