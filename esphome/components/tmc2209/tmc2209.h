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

class TMC2209Stepper;  // Forward declare

static TMC2209Stepper *components[TMC2209_NUM_COMPONENTS];
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
  bool *diag_triggered_ptr{nullptr};
  bool *driver_is_enabled_ptr{nullptr};

  void stop();

  static void index_isr(TMC2209ISRStore *arg);
  static void diag_isr(TMC2209ISRStore *arg);
};

class TMC2209Stepper : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2209Stepper(uint8_t address, bool use_internal_rsense, float rsense_);

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; }

  void enable(bool enable = true);
  void stop();

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  void rms_current(uint16_t mA);
  uint16_t rms_current();
  void rms_current_hold_scale(float scale);
  float rms_current_hold_scale();
  float motor_load();

  void add_on_stall_callback(std::function<void()> callback) { this->on_stall_callback_.add(std::move(callback)); }

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
  void chopconf_vsense(bool high_sensitivity);
  bool chopconf_vsense();
  void stallguard_sgthrs(uint8_t threshold);
  uint8_t stallguard_sgthrs();
  uint16_t stallguard_sgresult();
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  int16_t current_a();
  int16_t current_b();
  void vactual(int32_t velocity);
  uint32_t tstep();
  void ihold_irun_ihold(uint8_t current);
  uint8_t ihold_irun_ihold();
  void ihold_irun_irun(uint8_t current);
  uint8_t ihold_irun_irun();
  void ihold_irun_ihold_delay(uint8_t current);
  uint8_t ihold_irun_ihold_delay();
  void tpowerdown(uint8_t delay);
  uint8_t tpowerdown();

 protected:
  // TMC-API handlers
  uint8_t channel_{0};  // used for tmcapi channel index and esphome global component index
  uint8_t address_{0x00};
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  void update_registers_();
  bool reset_();
  bool restore_();

  void set_rms_current_();
  uint16_t current_scale_to_rms_current_(uint8_t current_scaling);

  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;
  InternalGPIOPin *enn_pin_;

  bool use_internal_rsense_;
  float rsense_;

  bool driver_is_enabled_{false};
  bool diag_triggered_{false};
  bool overtemp_detected_{false};
  uint32_t coolstep_tcoolthrs_{0};
  uint8_t stallguard_sgthrs_{0};
  uint16_t rms_current_{UINT16_MAX};
  float rms_current_hold_scale_{1.0};

  bool scheduled_powerdown_{false};
  bool prev_has_reached_target_{false};

  TMC2209ISRStore isr_store_{};
  Direction direction_{Direction::NONE};
  HighFrequencyLoopRequester high_freq_;
  CallbackManager<void()> on_stall_callback_;
};

template<typename... Ts> class TMC2209StepperConfigureAction : public Action<Ts...>, public Parented<TMC2209Stepper> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(uint, rms_current)
  TEMPLATABLE_VALUE(float, rms_current_hold_scale)
  TEMPLATABLE_VALUE(int, hold_current_delay)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->gconf_shaft(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->rms_current_.has_value())
      this->parent_->rms_current(this->rms_current_.value(x...));

    if (this->rms_current_hold_scale_.has_value())
      this->parent_->rms_current_hold_scale(this->rms_current_hold_scale_.value(x...));

    if (this->hold_current_delay_.has_value())
      this->parent_->ihold_irun_ihold_delay(this->hold_current_delay_.value(x...));

    if (this->coolstep_tcoolthrs_.has_value())
      this->parent_->coolstep_tcoolthrs(this->coolstep_tcoolthrs_.value(x...));

    if (this->stallguard_sgthrs_.has_value())
      this->parent_->stallguard_sgthrs(this->stallguard_sgthrs_.value(x...));
  }
};

template<typename... Ts> class TMC2209StepperStopAction : public Action<Ts...>, public Parented<TMC2209Stepper> {
 public:
  void play(Ts... x) override { this->parent_->stop(); }
};

class TMC2209StepperOnStallTrigger : public Trigger<> {
 public:
  explicit TMC2209StepperOnStallTrigger(TMC2209Stepper *parent) {
    parent->add_on_stall_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc
}  // namespace esphome
