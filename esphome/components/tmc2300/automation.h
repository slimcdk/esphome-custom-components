#pragma once
#include "tmc2300_api_registers.h"
#include "tmc2300_component.h"
#include "events.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace tmc2300 {

template<typename... Ts> class ConfigureAction : public Action<Ts...>, public Parented<TMC2300Component> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(bool, microstep_interpolation)
  TEMPLATABLE_VALUE(bool, enable_spreadcycle)
  TEMPLATABLE_VALUE(int, tcool_threshold)
  TEMPLATABLE_VALUE(int, tpwm_threshold)

  void const Ts &...x override {
    if (this->inverse_direction_.has_value())
      this->parent_->write_field(SHAFT_FIELD, this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->microstep_interpolation_.has_value())
      this->parent_->write_field(INTPOL_FIELD, this->microstep_interpolation_.value(x...));

    if (this->enable_spreadcycle_.has_value())
      this->parent_->write_field(EN_SPREADCYCLE_FIELD, this->enable_spreadcycle_.value(x...));

    if (this->tcool_threshold_.has_value())
      this->parent_->write_register(TCOOLTHRS, this->tcool_threshold_.value(x...));

    if (this->tpwm_threshold_.has_value())
      this->parent_->write_field(TPWMTHRS_FIELD, this->tpwm_threshold_.value(x...));
  }
};

template<typename... Ts> class ActivationAction : public Action<Ts...>, public Parented<TMC2300Component> {
  TEMPLATABLE_VALUE(bool, activate)

  void const Ts &...x override {
    if (this->activate_.has_value()) {
      this->parent_->enable(this->activate_.value(x...));
    }
  }
};

template<typename... Ts> class CurrentsAction : public Action<Ts...>, public Parented<TMC2300Component> {
 public:
  TEMPLATABLE_VALUE(int, standstill_mode)
  TEMPLATABLE_VALUE(uint8_t, irun)
  TEMPLATABLE_VALUE(uint8_t, ihold)
  TEMPLATABLE_VALUE(int, iholddelay)
  TEMPLATABLE_VALUE(int, tpowerdown)
  TEMPLATABLE_VALUE(float, run_current)
  TEMPLATABLE_VALUE(float, hold_current)

  void const Ts &...x override {
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

template<typename... Ts> class StallGuardAction : public Action<Ts...>, public Parented<TMC2300Component> {
 public:
  TEMPLATABLE_VALUE(int32_t, stallguard_threshold)

  void const Ts &...x override {
    if (this->stallguard_threshold_.has_value())
      this->parent_->write_register(SGTHRS, this->stallguard_threshold_.value(x...));
  }
};

template<typename... Ts> class CoolConfAction : public Action<Ts...>, public Parented<TMC2300Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, seimin)
  TEMPLATABLE_VALUE(uint8_t, semax)
  TEMPLATABLE_VALUE(uint8_t, semin)
  TEMPLATABLE_VALUE(uint8_t, sedn)
  TEMPLATABLE_VALUE(uint8_t, seup)

  void const Ts &...x override {
    if (this->seimin_.has_value())
      this->parent_->write_field(SEIMIN_FIELD, this->seimin_.value(x...));

    if (this->semax_.has_value())
      this->parent_->write_field(SEMAX_FIELD, this->semax_.value(x...));

    if (this->semin_.has_value())
      this->parent_->write_field(SEMIN_FIELD, this->semin_.value(x...));

    if (this->sedn_.has_value())
      this->parent_->write_field(SEDN_FIELD, this->sedn_.value(x...));

    if (this->seup_.has_value())
      this->parent_->write_field(SEUP_FIELD, this->seup_.value(x...));
  }
};

template<typename... Ts> class ChopConfAction : public Action<Ts...>, public Parented<TMC2300Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, tbl)
  TEMPLATABLE_VALUE(uint8_t, hend)
  TEMPLATABLE_VALUE(uint8_t, hstrt)

  void const Ts &...x override {
    if (this->tbl_.has_value())
      this->parent_->write_field(TBL_FIELD, this->tbl_.value(x...));

    if (this->hend_.has_value())
      this->parent_->write_field(HEND_FIELD, this->hend_.value(x...));

    if (this->hstrt_.has_value())
      this->parent_->write_field(HSTRT_FIELD, this->hstrt_.value(x...));
  }
};

template<typename... Ts> class PWMConfAction : public Action<Ts...>, public Parented<TMC2300Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, pwmlim)
  TEMPLATABLE_VALUE(uint8_t, pwmreg)
  TEMPLATABLE_VALUE(uint8_t, pwmautograd)
  TEMPLATABLE_VALUE(uint8_t, pwmautoscale)
  TEMPLATABLE_VALUE(uint8_t, pwmfreq)
  TEMPLATABLE_VALUE(uint8_t, pwmgrad)
  TEMPLATABLE_VALUE(uint8_t, pwmofs)

  void const Ts &...x override {
    if (this->pwmlim_.has_value())
      this->parent_->write_field(PWM_LIM_FIELD, this->pwmlim_.value(x...));

    if (this->pwmreg_.has_value())
      this->parent_->write_field(PWM_REG_FIELD, this->pwmreg_.value(x...));

    if (this->pwmautograd_.has_value())
      this->parent_->write_field(PWM_AUTOGRAD_FIELD, this->pwmautograd_.value(x...));

    if (this->pwmautoscale_.has_value())
      this->parent_->write_field(PWM_AUTOSCALE_FIELD, this->pwmautoscale_.value(x...));

    if (this->pwmfreq_.has_value())
      this->parent_->write_field(PWM_FREQ_FIELD, this->pwmfreq_.value(x...));

    if (this->pwmgrad_.has_value())
      this->parent_->write_field(PWM_GRAD_FIELD, this->pwmgrad_.value(x...));

    if (this->pwmofs_.has_value())
      this->parent_->write_field(PWM_OFS_FIELD, this->pwmofs_.value(x...));
  }
};

class OnDriverStatusTrigger : public Trigger<DriverStatusEvent> {
 public:
  explicit OnDriverStatusTrigger(TMC2300Component *parent) {
    parent->add_on_driver_status_callback([this](DriverStatusEvent code) { this->trigger(code); });
  }
};

class OnStallTrigger : public Trigger<> {
 public:
  explicit OnStallTrigger(TMC2300Component *parent) {
    parent->add_on_stall_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc2300
}  // namespace esphome
