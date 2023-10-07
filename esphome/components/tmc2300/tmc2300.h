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

struct TMC2300StepperDiagStore {
  bool fault_detected_{false};
  static void gpio_isr(TMC2300StepperDiagStore *arg);
};

class TMC2300Stepper : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2300Stepper();

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  void add_on_fault_signal_callback(std::function<void()> callback) {
    this->on_fault_signal_callback_.add(std::move(callback));
  }

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
  void ihold_irun_ihold(int32_t current);
  void ihold_irun_irun(int32_t current);
  void ihold_irun_ihold_delay(int32_t current);

  void chopconf_mres(uint8_t index);
  uint8_t chopconf_mres();
  void chopconf_blank_time(uint8_t select);

  //  protected:
  // TMC-API handlers
  uint8_t index_{0};  // used for tmcapi channel index and esphome global component index
  TMC2300TypeDef driver_;
  ConfigurationTypeDef config_;

  void update_registers_();
  static void IRAM_ATTR HOT diag_isr(TMC2300Stepper *stepper_);

  InternalGPIOPin *diag_pin_;
  CallbackManager<void()> on_fault_signal_callback_;

  uint8_t stallguard_sgthrs_;
  int32_t coolstep_tcoolthrs_;

  TMC2300StepperDiagStore diag_store_{};
};

class TMC2300StepperFaultSignalTrigger : public Trigger<> {
 public:
  explicit TMC2300StepperFaultSignalTrigger(TMC2300Stepper *parent) {
    parent->add_on_fault_signal_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc
}  // namespace esphome
