#pragma once

#include "esphome/core/automation.h"

#include "tmc2209.h"

namespace esphome {
namespace tmc2209 {

template<typename... Ts> class TMC2209ConfigureAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(bool, microstep_interpolation)
  TEMPLATABLE_VALUE(float, rms_current)
  TEMPLATABLE_VALUE(float, rms_current_hold_scale)
  TEMPLATABLE_VALUE(int, hold_current_delay)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->write_gconf_shaft(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->microstep_interpolation_.has_value())
      this->parent_->write_chopconf_intpol(this->microstep_interpolation_.value(x...));

    if (this->rms_current_.has_value())
      this->parent_->set_rms_current(this->rms_current_.value(x...));

    if (this->rms_current_hold_scale_.has_value())
      this->parent_->rms_current_hold_scale(this->rms_current_hold_scale_.value(x...));

    // if (this->hold_current_delay_.has_value())
    //   this->parent_->set_ihold_irun_ihold_delay(this->hold_current_delay_.value(x...));

    if (this->coolstep_tcoolthrs_.has_value())
      this->parent_->write_coolstep_tcoolthrs(this->coolstep_tcoolthrs_.value(x...));

    if (this->stallguard_sgthrs_.has_value())
      this->parent_->write_stallguard_sgthrs(this->stallguard_sgthrs_.value(x...));
  }
};

class TMC2209OnAlertTrigger : public Trigger<DriverEvent> {
 public:
  explicit TMC2209OnAlertTrigger(TMC2209 *parent) {
    parent->add_on_alert_callback([this](DriverEvent event) { this->trigger(event); });
  }
};

}  // namespace tmc2209
}  // namespace esphome
