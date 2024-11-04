#pragma once
#include "tmc2209_api_registers.h"
#include "tmc2209_component.h"
#include "events.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace tmc2209 {

template<typename... Ts> class ConfigureAction : public Action<Ts...>, public Parented<TMC2209Component> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(bool, microstep_interpolation)
  TEMPLATABLE_VALUE(bool, enable_spreadcycle)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->write_field(SHAFT_FIELD, this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->microstep_interpolation_.has_value())
      this->parent_->write_field(INTPOL_FIELD, this->microstep_interpolation_.value(x...));

    if (this->enable_spreadcycle_.has_value())
      this->parent_->write_field(EN_SPREADCYCLE_FIELD, this->enable_spreadcycle_.value(x...));
  }
};

template<typename... Ts> class CurrentsAction : public Action<Ts...>, public Parented<TMC2209Component> {
 public:
  TEMPLATABLE_VALUE(int, standstill_mode)
  TEMPLATABLE_VALUE(uint8_t, irun)
  TEMPLATABLE_VALUE(uint8_t, ihold)
  TEMPLATABLE_VALUE(int, iholddelay)
  TEMPLATABLE_VALUE(int, tpowerdown)
  TEMPLATABLE_VALUE(float, run_current)
  TEMPLATABLE_VALUE(float, hold_current)

  void play(Ts... x) override {
    if (this->standstill_mode_.has_value()) {
      if (this->parent_->read_field(EN_SPREADCYCLE_FIELD)) {
        ESP_LOGW(TAG, "standstill modes are only possible with StealthChop enabled.");
      }
      this->parent_->write_field(FREEWHEEL_FIELD, this->standstill_mode_.value(x...));
    }

    if (this->iholddelay_.has_value())
      this->parent_->write_field(IHOLDDELAY_FIELD, this->iholddelay_.value(x...));

    if (this->tpowerdown_.has_value())
      this->parent_->write_field(TPOWERDOWN_FIELD, this->tpowerdown_.value(x...));

    if (this->irun_.has_value())
      this->parent_->write_field(IRUN_FIELD, this->irun_.value(x...));

    if (this->ihold_.has_value())
      this->parent_->write_field(IHOLD_FIELD, this->ihold_.value(x...));

    if (this->run_current_.has_value())
      this->parent_->write_run_current(this->run_current_.value(x...));

    if (this->hold_current_.has_value())
      this->parent_->write_hold_current(this->hold_current_.value(x...));
  }
};

template<typename... Ts> class CoolstepAction : public Action<Ts...>, public Parented<TMC2209Component> {
 public:
  TEMPLATABLE_VALUE(int, tcool_threshold)

  void play(Ts... x) override {
    if (this->tcool_threshold_.has_value())
      this->parent_->write_register(TCOOLTHRS, this->tcool_threshold_.value(x...));
  }
};

class OnDriverStatusTrigger : public Trigger<DriverStatusEvent> {
 public:
  explicit OnDriverStatusTrigger(TMC2209Component *parent) {
    parent->add_on_driver_status_callback([this](DriverStatusEvent code) { this->trigger(code); });
  }
};

}  // namespace tmc2209
}  // namespace esphome
