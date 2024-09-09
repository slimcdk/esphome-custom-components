#pragma once

#include <limits>
#include <string>
#include <tuple>
#include <iostream>

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
// #include "esphome/core/automation.h"

#include "esphome/components/uart/uart.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

namespace esphome {
namespace tmc2209 {

#define TMC2209_IC_VERSION_33 0x21
#define DRIVER_STATE_TIMER_NAME "powerdown"
#define INDEX_FB_CHECK_TIMER_NAME "indexcheck"

enum DriverEvent {

  DIAG_TRIGGERED,
  STALLED,
  // CHARGPUMP_UNDERVOLTAGE,
  // SHORT_CIRCUIT,

  TEMPERATURE_NORMAL,
  OVERTEMPERATURE_PREWARNING,
  OVERTEMPERATURE_PREWARNING_CLEARED,
  OVERTEMPERATURE,
  OVERTEMPERATURE_CLEARED,
  TEMPERATURE_BELOW_120C,
  TEMPERATURE_ABOVE_120C,
  TEMPERATURE_BELOW_143C,
  TEMPERATURE_ABOVE_143C,
  TEMPERATURE_BELOW_150C,
  TEMPERATURE_ABOVE_150C,
  TEMPERATURE_BELOW_157C,
  TEMPERATURE_ABOVE_157C,
  A_OPEN_LOAD,
  A_OPEN_LOAD_CLEARED,
  B_OPEN_LOAD,
  B_OPEN_LOAD_CLEARED,
  A_LOW_SIDE_SHORT,
  A_LOW_SIDE_SHORT_CLEARED,
  B_LOW_SIDE_SHORT,
  B_LOW_SIDE_SHORT_CLEARED,
  A_GROUND_SHORT,
  A_GROUND_SHORT_CLEARED,
  B_GROUND_SHORT,
  B_GROUND_SHORT_CLEARED,
};

class TMC2209;  // forwared declare

static TMC2209 *components[TMC2209_NUM_COMPONENTS];
static uint16_t component_index = 0;

struct ISRStore {
  bool *pin_triggered_ptr{nullptr};
  static void IRAM_ATTR HOT pin_isr(ISRStore *arg) { (*(arg->pin_triggered_ptr)) = true; }
};

class EventHandler {
 public:
  EventHandler() = default;

  void set_on_rise_callback(std::function<void()> &&callback) { this->callback_rise_ = std::move(callback); }
  void set_on_fall_callback(std::function<void()> &&callback) { this->callback_fall_ = std::move(callback); }
  void set_callbacks(std::function<void()> &&callback_on_rise, std::function<void()> &&callback_on_fall) {
    this->callback_rise_ = std::move(callback_on_rise);
    this->callback_fall_ = std::move(callback_on_fall);
  }

  // Check state and trigger appropriate callbacks
  void check(bool state) {
    if (state != prev_) {
      if (state) {
        if (this->callback_rise_) {
          this->callback_rise_();
        }
      } else {
        if (this->callback_fall_) {
          this->callback_fall_();
        }
      }
      prev_ = state;  // Update previous state only when state changes
    }
  }

 private:
  bool prev_{false};
  std::function<void()> callback_rise_;
  std::function<void()> callback_fall_;
};

class TMC2209 : public Component, public uart::UARTDevice {
 public:
  TMC2209(uint8_t address, uint32_t clock_frequency);
  friend class TMC2209Stepper;

  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };

  void set_rsense(float resistance = 0, bool use_internal = false) {
    this->rsense_ = resistance;
    this->use_internal_rsense_ = use_internal;
  };

  void set_ottrim(uint8_t ottrim) { this->ottrim_ = ottrim; };

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  uint8_t get_address() { return this->address_; }

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  void set_rms_current(float A);
  float get_rms_current();
  void rms_current_hold_scale(float scale);
  float rms_current_hold_scale();
  float get_motor_load();

  void ihold_irun_ihold_delay_ms(uint32_t delay_in_ms);
  uint32_t ihold_irun_ihold_delay_ms();

  void tpowerdown_ms(uint32_t delay_in_ms);
  uint32_t tpowerdown_ms();
  std::tuple<uint8_t, uint8_t> unpack_ottrim_values(uint8_t ottrim);

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

  void write_gconf_iscale_analog(bool use_vref);
  void write_gconf_internal_rsense(bool internal);
  void write_gconf_en_spreadcycle(bool enable);
  void write_gconf_shaft(bool inverse);
  void write_gconf_index_otpw(bool use_otpw);
  void write_gconf_index_step(bool enable);
  void write_gconf_pdn_disable(bool disable);
  void write_gconf_mstep_reg_select(bool use);
  void write_chopconf_mres(uint8_t index);
  uint8_t read_chopconf_mres();
  void write_gconf_multistep_filt(bool enable);
  void write_gconf_test_mode(bool enable);
  uint32_t read_drv_status();
  bool read_gconf_index_otpw();
  bool read_ioin_diag();
  int8_t read_ioin_chip_version();
  void write_coolstep_tcoolthrs(int32_t threshold);
  void write_chopconf_intpol(bool enable);
  void write_chopconf_vsense(bool high_sensitivity);
  bool read_chopconf_vsense();
  void write_stallguard_sgthrs(uint8_t threshold);
  uint8_t read_stallguard_sgthrs();
  uint16_t read_stallguard_sgresult();
  void write_vactual(int32_t velocity);
  void write_ihold_irun_ihold(uint8_t current);
  void write_ihold_irun_irun(uint8_t current);
  uint8_t read_ihold_irun_irun();
  void write_ihold_irun_ihold_delay(uint8_t clock_cycle_factor);
  uint8_t read_ihold_irun_ihold_delay();
  void write_tpowerdown(uint8_t delay);
  uint8_t read_tpowerdown();

  void write_ottrim(uint8_t ottrim);
  uint8_t read_ottrim();

 protected:
  InternalGPIOPin *diag_pin_;

  uint16_t id_;  // used for tmcapi id index and esphome global component index
  uint8_t address_;
  optional<uint8_t> ottrim_;

  void set_rms_current_();
  float current_scale_to_rms_current_(uint8_t current_scaling);

  bool use_internal_rsense_;
  float rsense_;
  const uint32_t clock_frequency_;

  float rms_current_{std::numeric_limits<float>::max()};
  float rms_current_hold_scale_{1.0};

  ISRStore diag_isr_store_{};
  bool diag_triggered_{false};

  /* Driver event handlers */
  void setup_event_handlers();
  void handle_diag_rise_event();

  EventHandler diag_handler_{};  // Event on DIAG
  // EventHandler stst_handler_{};     // standstill indicator
  // EventHandler stealth_handler_{};  // StealthChop indicator (0=SpreadCycle mode, 1=StealthChop mode)
  EventHandler otpw_handler_{};   // overtemperature prewarning flag (Selected limit has been reached)
  EventHandler ot_handler_{};     // overtemperature flag (Selected limit has been reached)
  EventHandler t120_handler_{};   // 120°C comparator (Temperature threshold is exceeded)
  EventHandler t143_handler_{};   // 143°C comparator (Temperature threshold is exceeded)
  EventHandler t150_handler_{};   // 150°C comparator (Temperature threshold is exceeded)
  EventHandler t157_handler_{};   // 157°C comparator (Temperature threshold is exceeded)
  EventHandler olb_handler_{};    // open load indicator phase B
  EventHandler ola_handler_{};    // open load indicator phase B
  EventHandler s2vsb_handler_{};  // low side short indicator phase B
  EventHandler s2vsa_handler_{};  // low side short indicator phase A
  EventHandler s2gb_handler_{};   // short to ground indicator phase B
  EventHandler s2ga_handler_{};   // short to ground indicator phase A
  /* */

  CallbackManager<void(const DriverEvent &event)> on_alert_callback_;
  HighFrequencyLoopRequester high_freq_;
};

}  // namespace tmc2209
}  // namespace esphome
