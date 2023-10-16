#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/stepper/stepper.h"

extern "C" {
#include <ic/TMC2300/TMC2300.h>
}

namespace esphome {
namespace tmc {

#define MAX_ALLOWED_COMPONENTS 1

class TMC2300Stepper;  // Forward declare

static TMC2300Stepper *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2300_stepper_global_index = 0;

enum Direction : int8_t {
  CLOCKWISE = -1,     // Moving one direction
  NONE = 0,           // Not moving
  ANTICLOCKWISE = 1,  // Moving the other direction
};

struct TMC2300ISRStore {
  ISRInternalGPIOPin enn_pin_;

  int32_t *current_position_ptr{nullptr};
  int32_t *target_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  bool *diag_triggered_ptr{nullptr};
  bool *driver_is_enabled_ptr{nullptr};

  void stop();

  static void index_isr(TMC2300ISRStore *arg);
  static void diag_isr(TMC2300ISRStore *arg);
};

class TMC2300Stepper : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2300Stepper(uint8_t address, bool use_internal_rsense, float resistance);

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }

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
  uint16_t gconf();
  void gconf(uint16_t setting);

  void gconf_en_spreadcycle(bool enable);
  bool gconf_en_spreadcycle();
  bool gconf_shaft();
  void gconf_shaft(bool inverse);
  void gconf_pdn_disable(bool disable);
  bool gconf_pdn_disable();
  void gconf_mstep_reg_select(bool use);
  bool gconf_mstep_reg_select();
  void gconf_multistep_filt(bool enable);
  bool gconf_multistep_filt();
  void gconf_test_mode(bool enable);
  bool gconf_test_mode();
  void gconf_diag_step(bool enable);
  bool gconf_diag_step();
  void gconf_diag_index(bool enable);
  bool gconf_diag_index();

  uint16_t gstat();
  void gstat(uint16_t setting);
  bool gstat_reset();
  void gstat_reset(bool clear);
  bool gstat_drv_err();
  void gstat_drv_err(bool clear);

  uint8_t ifcnt();
  uint8_t slave_conf();

  uint32_t ioin();
  bool ioin_en();
  bool ioin_nstdby();
  bool ioin_diag();
  bool ioin_stepper();
  bool ioin_pdn_uart();
  bool ioin_mode();
  bool ioin_step();
  bool ioin_dir();
  bool ioin_comp_a1a2();
  bool ioin_comp_b1b2();
  int8_t ioin_version();

  void coolstep_tcoolthrs(int32_t threshold);
  int32_t coolstep_tcoolthrs();
  void stallguard_sgthrs(uint8_t threshold);
  uint8_t stallguard_sgthrs();
  uint16_t stallguard_sgresult();
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  void vactual(int32_t velocity);

  void ihold_irun_ihold(uint8_t current);
  uint8_t ihold_irun_ihold();
  void ihold_irun_irun(uint8_t current);
  uint8_t ihold_irun_irun();
  void ihold_irun_ihold_delay(uint8_t current);
  uint8_t ihold_irun_ihold_delay();

  void chopconf_mres(uint8_t index);
  uint8_t chopconf_mres();
  void chopconf_blank_time(uint8_t select);

 protected:
  // TMC-API handlers
  uint8_t channel_{0};  // used for tmcapi channel index and esphome global component index
  uint8_t address_{0x00};

  TMC2300TypeDef driver_;
  ConfigurationTypeDef config_;
  void update_registers_();
  bool reset_();
  bool restore_();

  void set_rms_current_();
  uint16_t current_scale_to_rms_current_(uint8_t current_scaling);

  InternalGPIOPin *enn_pin_;
  InternalGPIOPin *diag_pin_;

  bool driver_is_enabled_{false};
  bool use_internal_rsense_;
  float rsense_;
  uint32_t coolstep_tcoolthrs_{0};
  uint8_t stallguard_sgthrs_{0};
  uint16_t rms_current_{UINT16_MAX};
  float rms_current_hold_scale_{1.0};

  TMC2300ISRStore isr_store_{};
  Direction direction_{Direction::NONE};
  HighFrequencyLoopRequester high_freq_;
  CallbackManager<void()> on_stall_callback_;
};

template<typename... Ts> class TMC2300StepperConfigureAction : public Action<Ts...>, public Parented<TMC2300Stepper> {
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

template<typename... Ts> class TMC2300StepperStopAction : public Action<Ts...>, public Parented<TMC2300Stepper> {
 public:
  void play(Ts... x) override { this->parent_->stop(); }
};

class TMC2300StepperOnStallTrigger : public Trigger<> {
 public:
  explicit TMC2300StepperOnStallTrigger(TMC2300Stepper *parent) {
    parent->add_on_stall_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc
}  // namespace esphome
