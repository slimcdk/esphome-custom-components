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

}  // namespace tmc2209
}  // namespace esphome
