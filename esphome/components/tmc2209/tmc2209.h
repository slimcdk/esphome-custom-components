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

/*
class Latch {
 public:
  Latch() = default;

  bool check(bool state) {
    const bool trigger = (state && state != prevState_);
    prevState_ = state;
    return trigger;
  }

 private:
  bool prevState_{false};
};
*/

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

enum Direction : int8_t {
  CLOCKWISE = -1,     // Moving one direction
  NONE = 0,           // Not moving
  ANTICLOCKWISE = 1,  // Moving the other direction
};

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

  void rms_current(float A);
  float rms_current();
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
  // Register (all fields)
  void write_register(uint8_t address, int32_t value) { tmc2209_writeRegister(this->id_, address, value); }
  int32_t read_register(uint8_t address) { return tmc2209_readRegister(this->id_, address); }

  // Register field (single field within register)
  void write_field(RegisterField field, uint32_t value) { tmc2209_fieldWrite(this->id_, field, value); }
  uint32_t read_field(RegisterField field) { return tmc2209_fieldRead(this->id_, field); }

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
  void chopconf_intpol(bool enable);
  bool chopconf_intpol();
  void chopconf_vsense(bool high_sensitivity);
  bool chopconf_vsense();
  void stallguard_sgthrs(uint8_t threshold);
  uint8_t stallguard_sgthrs();
  uint16_t stallguard_sgresult();
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  int16_t current_a();
  int16_t current_b();
  int32_t vactual();
  void vactual(int32_t velocity);
  uint32_t tstep();
  void ihold_irun_ihold(uint8_t current);
  uint8_t ihold_irun_ihold();
  void ihold_irun_irun(uint8_t current);
  uint8_t ihold_irun_irun();
  void ihold_irun_ihold_delay(uint8_t clock_cycle_factor);
  uint8_t ihold_irun_ihold_delay();
  void tpowerdown(uint8_t delay);
  uint8_t tpowerdown();

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

  uint32_t coolstep_tcoolthrs_{0};
  uint8_t stallguard_sgthrs_{0};
  float rms_current_{std::numeric_limits<float>::max()};
  float rms_current_hold_scale_{1.0};

  ISRStore index_isr_store_{};
  ISRStore diag_isr_store_{};
  bool index_triggered_{false};
  bool diag_triggered_{false};

  void configure_event_handlers();
  void run_event_triggers();
  void handle_diag_event();
  void handle_index_event();

  // helper flags to trigger callbacks / automations

  // Driver event handlers
  EventHandler index_handler_{};  // Events on INDEX
  EventHandler diag_handler_{};   // Events on DIAG
  EventHandler nt_handler_{};     // Normal temperature
  EventHandler otpw_handler_{};   // Overtemperature prewarning
  EventHandler ot_handler_{};     // Overtemperature
  EventHandler t120_handler_{};   // Temperature above 120C
  EventHandler t143_handler_{};   // Temperature above 143C
  EventHandler t150_handler_{};   // Temperature above 150C
  EventHandler t157_handler_{};   // Temperature above 157C

  CallbackManager<void(DriverEvent)> on_alert_callback_{};
};

class TMC2209OnAlertTrigger : public Trigger<DriverEvent> {
 public:
  explicit TMC2209OnAlertTrigger(TMC2209 *parent) {
    parent->add_on_alert_callback([this](DriverEvent event) { this->trigger(event); });
  }
};

template<typename... Ts> class TMC2209ConfigureAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(bool, microstep_interpolation)
  TEMPLATABLE_VALUE(float, rms_current)
  TEMPLATABLE_VALUE(float, rms_current_hold_scale)
  TEMPLATABLE_VALUE(int, hold_current_delay)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->gconf_shaft(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->microstep_interpolation_.has_value())
      this->parent_->chopconf_intpol(this->microstep_interpolation_.value(x...));

    if (this->rms_current_.has_value())
      this->parent_->rms_current(this->rms_current_.value(x...));

    if (this->rms_current_hold_scale_.has_value())
      this->parent_->rms_current_hold_scale(this->rms_current_hold_scale_.value(x...));

    // if (this->hold_current_delay_.has_value())
    //   this->parent_->ihold_irun_ihold_delay(this->hold_current_delay_.value(x...));

    if (this->coolstep_tcoolthrs_.has_value())
      this->parent_->coolstep_tcoolthrs(this->coolstep_tcoolthrs_.value(x...));

    if (this->stallguard_sgthrs_.has_value())
      this->parent_->stallguard_sgthrs(this->stallguard_sgthrs_.value(x...));
  }
};

}  // namespace tmc2209
}  // namespace esphome
