#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/pulse_counter/pulse_counter_sensor.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

namespace esphome {
namespace tmc {

#ifdef HAS_PCNT
#define USE_PCNT true
#else
#define USE_PCNT false
#endif

#define MAX_ALLOWED_COMPONENTS 3
#define TMC2209_DEFAULT_CHIP_VERSION 0x21

class TMC2209Stepper;  // Forward declare

static TMC2209Stepper *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_stepper_global_channel_index = 0;

struct IndexStore {
  volatile int32_t current_{0};
  int32_t target_{0};
  volatile bool target_reached_{true};
  static void pin_intr(IndexStore *arg);
};

class TMC2209Stepper : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2209Stepper();

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps) override;
  void report_position(int32_t steps) override;
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; }

  // bool can_proceed() override { return this->cfg_state_ == CONFIG_READY; }

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  void blank_time(uint8_t select);
  uint16_t gconf();
  void gconf(uint16_t setting);
  void gconf_iscale_analog(bool use_vref);
  bool gconf_iscale_analog();
  void gconf_internal_rsense(bool use_internal);
  bool gconf_internal_rsense();
  void gconf_en_spreadcycle(bool enable);
  bool gconf_en_spreadcycle();
  bool gconf_inverse_direction();
  void gconf_inverse_direction(bool inverse_direction);
  bool gconf_index_otpw();
  void gconf_index_otpw(bool use_otpw);
  void gconf_index_step(bool enable);
  bool gconf_index_step();
  void gconf_pdn_disable(bool disable);
  bool gconf_pdn_disable();
  void gconf_mstep_reg_select(bool use);
  bool gconf_mstep_reg_select();
  void chopconf_mres(uint8_t index);
  uint8_t chopconf_mres();
  void gconf_multistep_filt(bool enable);
  bool gconf_multistep_filt();
  void gconf_test_mode(bool enable);
  bool gconf_test_mode();
  uint16_t gstat();
  void gstat(uint16_t setting);
  bool gstat_reset();
  void gstat_reset(bool clear);
  bool gstat_drv_err();
  void gstat_drv_err(bool clear);
  bool gstat_uv_cp();
  void gstat_uv_cp(bool clear);
  bool drv_status_stst();
  bool drv_status_stealth();
  uint8_t drv_status_cs_actual();
  bool drv_status_otpw();
  bool drv_status_ot();
  bool drv_status_t120();
  bool drv_status_t143();
  bool drv_status_t150();
  bool drv_status_t157();
  bool drv_status_ola();
  bool drv_status_olb();
  bool drv_status_s2vsa();
  bool drv_status_s2vsb();
  bool drv_status_s2ga();
  bool drv_status_s2gb();
  uint8_t transmission_counter();
  uint32_t ioin();
  bool ioin_enn();
  bool ioin_ms1();
  bool ioin_ms2();
  bool ioin_diag();
  bool ioin_pdn_uart();
  bool ioin_step();
  bool ioin_spread_en();
  bool ioin_dir();
  int8_t ioin_chip_version();
  uint32_t otpread();
  bool optread_en_spreadcycle();
  void fclktrim(uint8_t fclktrim);
  uint8_t fclktrim();
  void ottrim(uint8_t ottrim);
  uint8_t ottrim();
  void coolstep_tcoolthrs(int32_t threshold);
  int32_t coolstep_tcoolthrs();
  bool chopconf_dedge();
  void chopconf_dedge(bool set);
  void stallguard_sgthrs(uint8_t threshold);
  uint8_t stallguard_sgthrs();
  uint16_t stallguard_sgresult();
  float stallguard_load();
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  int16_t current_a();
  int16_t current_b();
  void vactual(int32_t velocity);
  void ihold_irun_ihold(int32_t current);
  void ihold_irun_irun(int32_t current);
  void ihold_irun_ihold_delay(int32_t current);

  void set_enn_pin(GPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }
  void set_address(uint8_t address) { this->address_ = address; }

  void enable(bool enable = true);
  void disable() { this->enable(false); };
  bool enabled() { return this->enn_pin_state_; }

  TMC2209TypeDef *get_driver() { return &this->driver_; };
  uint8_t get_address() { return this->address_; };
  void set_config_state(ConfigState state);

  void add_on_fault_signal_callback(std::function<void()> callback) {
    this->on_fault_signal_callback_.add(std::move(callback));
  }

  void stop();

  void velocity(int32_t velocity);

 protected:
  uint8_t channel_ = 0;
  uint8_t address_;
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  ConfigState cfg_state_;

  GPIOPin *enn_pin_;
  bool enn_pin_state_;
  bool moving_forward_;

  uint32_t coolstep_tcoolthrs_{0};
  uint8_t stallguard_sgthrs_{0};

  // Diag output tracking
  InternalGPIOPin *diag_pin_;
  static void diag_gpio_intr(TMC2209Stepper *driver_);
  bool fault_detected_{false};

  InternalGPIOPin *index_pin_;
  IndexStore index_store_{};

  CallbackManager<void()> on_fault_signal_callback_;

  void check_position_();
};

class TMC2209StepperFaultSignalTrigger : public Trigger<> {
 public:
  explicit TMC2209StepperFaultSignalTrigger(TMC2209Stepper *parent) {
    parent->add_on_fault_signal_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts> class TMC2209StepperConfigureAction : public Action<Ts...>, public Parented<TMC2209Stepper> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(int, velocity)
  TEMPLATABLE_VALUE(int, run_current)
  TEMPLATABLE_VALUE(int, hold_current)
  TEMPLATABLE_VALUE(int, hold_current_delay)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->gconf_inverse_direction(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->velocity_.has_value())
      this->parent_->vactual(this->velocity_.value(x...));

    if (this->run_current_.has_value())
      this->parent_->ihold_irun_irun(this->run_current_.value(x...));

    if (this->hold_current_.has_value())
      this->parent_->ihold_irun_ihold(this->hold_current_.value(x...));

    if (this->hold_current_delay_.has_value())
      this->parent_->ihold_irun_ihold_delay(this->hold_current_delay_.value(x...));

    if (this->coolstep_tcoolthrs_.has_value())
      this->parent_->coolstep_tcoolthrs(this->coolstep_tcoolthrs_.value(x...));

    if (this->stallguard_sgthrs_.has_value())
      this->parent_->stallguard_sgthrs(this->stallguard_sgthrs_.value(x...));
  }
};

}  // namespace tmc
}  // namespace esphome
