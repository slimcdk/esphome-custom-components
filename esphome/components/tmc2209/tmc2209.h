#pragma once

#include <limits>
#include <string>

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"

#include "esphome/components/uart/uart.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

namespace esphome {
namespace tmc2209 {

#define DRIVER_STATE_TIMER_NAME "powerdown"
#define INDEX_FB_CHECK_TIMER_NAME "indexcheck"

class EventHandler {
 public:
  EventHandler() = default;

  void set_callback(std::function<void()> &&callback) { this->callback_ = std::move(callback); }
  void set_callback_recover(std::function<void()> &&callback) { this->callback_recover_ = std::move(callback); }
  void set_callbacks(std::function<void()> &&callback, std::function<void()> &&callback_recover) {
    this->callback_ = std::move(callback);
    this->callback_recover_ = std::move(callback_recover);
  }

  // Check state and trigger appropriate callbacks
  void check(bool state) {
    if (state != prev_) {
      if (state) {
        if (this->callback_) {
          this->callback_();
        }
      } else {
        if (this->callback_recover_) {
          this->callback_recover_();
        }
      }
      prev_ = state;  // Update previous state only when state changes
    }
  }

 private:
  bool prev_{false};
  std::function<void()> callback_;
  std::function<void()> callback_recover_;
};

class TMC2209;  // Forward declare

static TMC2209 *components[TMC2209_NUM_COMPONENTS];
static uint16_t tmc2209_global_index = 0;

enum DriverEvent {

  INDEX_TRIGGERED,
  DIAG_TRIGGERED,
  STALLED,
  // CHARGPUMP_UNDERVOLTAGE,
  // SHORT_CIRCUIT,

  TEMPERATURE_NORMAL,
  OVERTEMPERATURE_PREWARNING,
  OVERTEMPERATURE_PREWARNING_GONE,
  OVERTEMPERATURE,
  OVERTEMPERATURE_GONE,
  TEMPERATURE_BELOW_120C,
  TEMPERATURE_ABOVE_120C,
  TEMPERATURE_BELOW_143C,
  TEMPERATURE_ABOVE_143C,
  TEMPERATURE_BELOW_150C,
  TEMPERATURE_ABOVE_150C,
  TEMPERATURE_BELOW_157C,
  TEMPERATURE_ABOVE_157C,
};

struct ISRStore {
  bool *pin_triggered_ptr{nullptr};
  static void IRAM_ATTR HOT pin_isr(ISRStore *arg) { (*(arg->pin_triggered_ptr)) = true; }
};

class TMC2209 : public Component, public uart::UARTDevice {
 public:
  TMC2209(uint8_t address);
  friend class TMC2209Stepper;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; };

  void set_oscillator_frequency(uint32_t frequency) { this->oscillator_freq_ = frequency; };
  void set_rsense(float resistance = 0, bool use_internal = false) {
    this->rsense_ = resistance;
    this->use_internal_rsense_ = use_internal;
  };

  uint8_t get_address();

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  void set_rms_current(float A);
  float get_rms_current();
  void rms_current_hold_scale(float scale);
  float rms_current_hold_scale();
  float motor_load();

  void ihold_irun_ihold_delay_ms(uint32_t delay_in_ms);
  uint32_t ihold_irun_ihold_delay_ms();

  void tpowerdown_ms(uint32_t delay_in_ms);
  uint32_t tpowerdown_ms();

  void add_on_alert_callback(std::function<void(DriverEvent)> &&callback) {
    this->on_alert_callback_.add(std::move(callback));
  }

  // TMC-API wrappers
  // Write or read a register (all fields)
  void write_register(uint8_t address, int32_t value) { tmc2209_writeRegister(this->id_, address, value); }
  int32_t read_register(uint8_t address) { return tmc2209_readRegister(this->id_, address); }

  // Write or read a register field (single field within register)
  void write_field(RegisterField field, uint32_t value) { tmc2209_fieldWrite(this->id_, field, value); }
  uint32_t read_field(RegisterField field) { return tmc2209_fieldRead(this->id_, field); }

  void set_gconf_iscale_analog(bool use_vref);
  void set_gconf_internal_rsense(bool internal);
  void set_gconf_en_spreadcycle(bool enable);
  void set_gconf_shaft(bool inverse);
  void set_gconf_index_otpw(bool use_otpw);
  void set_gconf_index_step(bool enable);
  void set_gconf_pdn_disable(bool disable);
  void set_gconf_mstep_reg_select(bool use);
  void set_chopconf_mres(uint8_t index);
  uint8_t get_chopconf_mres();
  void set_gconf_multistep_filt(bool enable);
  void set_gconf_test_mode(bool enable);
  bool get_drv_status_otpw();
  bool get_drv_status_ot();
  bool get_drv_status_t120();
  bool get_drv_status_t143();
  bool get_drv_status_t150();
  bool get_drv_status_t157();
  bool get_ioin_diag();
  int8_t get_ioin_chip_version();
  void set_coolstep_tcoolthrs(int32_t threshold);
  void set_chopconf_intpol(bool enable);
  void set_chopconf_vsense(bool high_sensitivity);
  bool get_chopconf_vsense();
  void set_stallguard_sgthrs(uint8_t threshold);
  uint8_t get_stallguard_sgthrs();
  uint16_t get_stallguard_sgresult();
  void set_vactual(int32_t velocity);
  void set_ihold_irun_ihold(uint8_t current);
  void set_ihold_irun_irun(uint8_t current);
  uint8_t get_ihold_irun_irun();
  void set_ihold_irun_ihold_delay(uint8_t clock_cycle_factor);
  uint8_t get_ihold_irun_ihold_delay();
  void set_tpowerdown(uint8_t delay);
  uint8_t get_tpowerdown();

 protected:
  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;

  uint16_t id_;  // used for tmcapi id index and esphome global component index
  uint8_t address_{0x00};

  void set_rms_current_();
  float current_scale_to_rms_current_(uint8_t current_scaling);

  bool use_internal_rsense_;
  float rsense_;
  uint32_t oscillator_freq_{12000000};

  float rms_current_{std::numeric_limits<float>::max()};
  float rms_current_hold_scale_{1.0};

  ISRStore index_isr_store_{};
  ISRStore diag_isr_store_{};
  bool index_triggered_{false};
  bool diag_triggered_{false};

  /* Driver event handlers */
  void configure_event_handlers();
  void run_event_triggers();
  void handle_diag_event();
  void handle_index_event();

  EventHandler index_handler_{};  // Events on INDEX
  EventHandler diag_handler_{};   // Events on DIAG
  EventHandler nt_handler_{};     // Normal temperature
  EventHandler otpw_handler_{};   // Overtemperature prewarning
  EventHandler ot_handler_{};     // Overtemperature
  EventHandler t120_handler_{};   // Temperature above 120C
  EventHandler t143_handler_{};   // Temperature above 143C
  EventHandler t150_handler_{};   // Temperature above 150C
  EventHandler t157_handler_{};   // Temperature above 157C
  /* */

  CallbackManager<void(DriverEvent)> on_alert_callback_{};
  HighFrequencyLoopRequester high_freq_;
};

}  // namespace tmc2209
}  // namespace esphome
