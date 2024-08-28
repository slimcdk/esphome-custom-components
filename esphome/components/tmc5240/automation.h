#pragma once

#include "esphome/core/automation.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240 {

template<typename... Ts> class TMC5240ConfigureAction : public Action<Ts...>, public Parented<TMC5240Stepper> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(float, rms_current)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->write_field(TMC5240_SHAFT_FIELD, this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));
  }
};

class TMC5240OnAlertTrigger : public Trigger<DriverEvent> {
 public:
  explicit TMC5240OnAlertTrigger(TMC5240Stepper *parent) {
    parent->add_on_alert_callback([this](DriverEvent event) { this->trigger(event); });
  }
};

}  // namespace tmc5240
}  // namespace esphome
