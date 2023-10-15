#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/stepper/stepper.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

namespace esphome {
namespace tmc {

#define MAX_ALLOWED_COMPONENTS 3

class TMC2209Stepper;  // Forward declare

static TMC2209Stepper *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_stepper_global_index = 0;

enum Direction : int8_t {
  CLOCKWISE = -1,     // Moving one direction
  NONE = 0,           // Not moving
  ANTICLOCKWISE = 1,  // Moving the other direction
};

struct TMC2209ISRStore {
  ISRInternalGPIOPin enn_pin_;

  int32_t *current_position_ptr{nullptr};
  int32_t *target_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  bool *stall_detected_ptr{nullptr};
  bool *driver_is_enabled_ptr{nullptr};

  void stop();

  static void pulse_isr(TMC2209ISRStore *arg);
  static void fault_isr(TMC2209ISRStore *arg);
};

class TMC2209Stepper : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2209Stepper(bool use_internal_rsense);

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  // void set_target(int32_t target) override;

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; }
  void set_rsense(float resistance) { this->resistance_ = resistance; }
  void set_address(uint8_t address) { this->address_ = address; }

  void enable(bool enable = true);
  void disable() { this->enable(false); };
  bool enabled() { return this->driver_is_enabled_; }
  void stop();

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  void add_on_fault_signal_callback(std::function<void()> callback) {
    this->on_fault_signal_callback_.add(std::move(callback));
  }

  // TMC-API wrappers
  void blank_time(uint8_t select);
  uint16_t gconf();
  void gconf(uint16_t setting);
  void gconf_iscale_analog(bool use_vref);
  bool gconf_iscale_analog();
  void gconf_internal_rsense(bool use_internal);
  bool gconf_internal_rsense();
  void gconf_en_spreadcycle(bool enable);
  bool gconf_en_spreadcycle();
  bool gconf_shaft();
  void gconf_shaft(bool inverse);
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
  float motor_load();
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  int16_t current_a();
  int16_t current_b();
  void vactual(int32_t velocity);
  void ihold_irun_ihold(int32_t current);
  void ihold_irun_irun(int32_t current);
  void ihold_irun_ihold_delay(int32_t current);

 protected:
  // TMC-API handlers
  uint8_t index_{0};  // used for tmcapi channel index and esphome global component index
  uint8_t address_{0x00};
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  void update_registers_();
  bool reset_();
  bool restore_();

  bool use_internal_rsense_;
  bool driver_is_enabled_{false};
  float resistance_{.11};
  bool stall_detected_{false};
  bool overtemp_detected_{false};
  uint32_t coolstep_tcoolthrs_{0};
  uint8_t stallguard_sgthrs_{0};

  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;
  InternalGPIOPin *enn_pin_;

  TMC2209ISRStore isr_store_{};
  Direction direction_{Direction::NONE};

  HighFrequencyLoopRequester high_freq_;
  CallbackManager<void()> on_fault_signal_callback_;

  time_t last_interval_time_{0};
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
  TEMPLATABLE_VALUE(int, run_current)
  TEMPLATABLE_VALUE(int, hold_current)
  TEMPLATABLE_VALUE(int, hold_current_delay)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->gconf_shaft(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

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

template<typename... Ts> class TMC2209StepperVelocityAction : public Action<Ts...>, public Parented<TMC2209Stepper> {
 public:
  TEMPLATABLE_VALUE(int, velocity)
  void play(Ts... x) override {
    // this->parent_->set_velocity(this->velocity_.value(x...));
  }
};

}  // namespace tmc
}  // namespace esphome
