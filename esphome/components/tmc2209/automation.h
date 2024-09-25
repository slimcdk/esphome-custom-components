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
  TEMPLATABLE_VALUE(float, run_current)
  TEMPLATABLE_VALUE(float, hold_current)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int32_t, stallguard_sgthrs)
  TEMPLATABLE_VALUE(int, standstill_mode)
  TEMPLATABLE_VALUE(uint8_t, irun)
  TEMPLATABLE_VALUE(uint8_t, ihold)
  TEMPLATABLE_VALUE(int, iholddelay)
  TEMPLATABLE_VALUE(int, tpowerdown)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->write_field(TMC2209_SHAFT_FIELD, this->inverse_direction_.value(x...));

    if (this->coolstep_tcoolthrs_.has_value())
      this->parent_->write_register(TMC2209_TCOOLTHRS, this->coolstep_tcoolthrs_.value(x...));

    if (this->stallguard_sgthrs_.has_value())
      this->parent_->write_register(TMC2209_SGTHRS, this->stallguard_sgthrs_.value(x...));

    if (this->microstep_interpolation_.has_value())
      this->parent_->write_field(TMC2209_INTPOL_FIELD, this->microstep_interpolation_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->standstill_mode_.has_value())
      this->parent_->write_field(TMC2209_FREEWHEEL_FIELD, this->standstill_mode_.value(x...));

    if (this->iholddelay_.has_value())
      this->parent_->write_field(TMC2209_IHOLDDELAY_FIELD, this->iholddelay_.value(x...));

    if (this->tpowerdown_.has_value())
      this->parent_->write_field(TMC2209_TPOWERDOWN_FIELD, this->tpowerdown_.value(x...));

    if (this->irun_.has_value())
      this->parent_->write_field(TMC2209_IRUN_FIELD, this->irun_.value(x...));

    if (this->ihold_.has_value())
      this->parent_->write_field(TMC2209_IHOLD_FIELD, this->ihold_.value(x...));

    if (this->run_current_.has_value())
      this->parent_->write_run_current(this->run_current_.value(x...));

    if (this->hold_current_.has_value())
      this->parent_->write_hold_current(this->hold_current_.value(x...));
  }
};

template<typename... Ts> class TMC2209StopAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, disable)

  void play(Ts... x) override {
    if (this->disable_.has_value()) {
      this->parent_->stop(this->disable_.value(x...));
    }
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
