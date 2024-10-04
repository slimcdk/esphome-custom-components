#pragma once

#include "tmc2209.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace tmc2209 {

template<typename... Ts> class ConfigureAction : public Action<Ts...>, public Parented<TMC2209> {
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
  TEMPLATABLE_VALUE(bool, enable_spreadcycle)
  TEMPLATABLE_VALUE(float, stall_detection_activation_level)

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

    if (this->standstill_mode_.has_value()) {
      if (this->parent_->read_field(TMC2209_EN_SPREADCYCLE_FIELD)) {
        ESP_LOGW(TAG, "standstill modes are only possible with StealthChop enabled.");
      }
      this->parent_->write_field(TMC2209_FREEWHEEL_FIELD, this->standstill_mode_.value(x...));
    }
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

    if (this->enable_spreadcycle_.has_value()) {
      this->parent_->write_field(TMC2209_EN_SPREADCYCLE_FIELD, this->enable_spreadcycle_.value(x...));
    }

    if (this->stall_detection_activation_level_.has_value()) {
      this->parent_->set_stall_detection_activation_level(this->stall_detection_activation_level_.value(x...));
    }
  }
};

template<typename... Ts> class ActivationAction : public Action<Ts...>, public Parented<TMC2209> {
  TEMPLATABLE_VALUE(bool, activate)

  void play(Ts... x) override {
    if (this->activate_.has_value()) {
      this->parent_->enable(this->activate_.value(x...));
    }
  }
};

class OnAlertTrigger : public Trigger<DriverEvent> {
 public:
  explicit OnAlertTrigger(TMC2209 *parent) {
    parent->add_on_alert_callback([this](DriverEvent event) { this->trigger(event); });
  }
};

}  // namespace tmc2209
}  // namespace esphome
