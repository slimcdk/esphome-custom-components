#pragma once

#include "tmc2209_api.h"
#include "tmc2209_api_registers.h"

// #include <limits>
// #include <string>
// #include <tuple>
// #include <iostream>

// #include "esphome/core/helpers.h"
// #include "esphome/core/component.h"
// #include "esphome/core/automation.h"

namespace esphome {
namespace tmc2209 {

#define VSENSE_HIGH 0.325f
#define VSENSE_LOW 0.180f

#define mA2A(mA) ((float) mA / 1000.0)
#define A2mA(A) (uint16_t)(A * 1000)

enum DriverEvent {
  DIAG_TRIGGERED,
  DIAG_TRIGGER_CLEARED,
  STALLED,
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
  CP_UNDERVOLTAGE,
  CP_UNDERVOLTAGE_CLEARED
};

struct ISRPinTriggerStore {
  bool *pin_triggered_ptr{nullptr};
  static void IRAM_ATTR HOT pin_isr(ISRPinTriggerStore *arg) { (*(arg->pin_triggered_ptr)) = true; }
};

class EventHandler {
 public:
  EventHandler(bool initial_state = false) : prev_(initial_state) {}

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
  bool prev_;
  std::function<void()> callback_rise_;
  std::function<void()> callback_fall_;
};

class TMC2209 : public TMC2209API {
 public:
  TMC2209(uint8_t address, uint32_t clk_frequency) : TMC2209API(address), clk_frequency_(clk_frequency){};

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; };
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; };
  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; };
  void set_dir_pin(GPIOPin *pin) { this->dir_pin_ = pin; };

  /* run and hold currents */
  float read_vsense();
  uint8_t rms_current_to_current_scale_mA_no_clamp(uint16_t mA);
  uint16_t current_scale_to_rms_current_mA(uint8_t cs);
  uint8_t rms_current_to_current_scale_mA(uint16_t mA);
  void write_run_current_mA(uint16_t mA);
  void write_hold_current_mA(uint16_t mA);
  uint16_t read_run_current_mA();
  uint16_t read_hold_current_mA();
  void write_run_current(float A) { this->write_run_current_mA(A2mA(A)); };
  void write_hold_current(float A) { this->write_hold_current_mA(A2mA(A)); };
  float read_run_current() { return mA2A(this->read_run_current_mA()); };
  float read_hold_current() { return mA2A(this->read_hold_current_mA()); };

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();
  float get_motor_load();
  bool is_stalled();
  void set_tpowerdown_ms(uint32_t delay_in_ms);
  uint32_t get_tpowerdown_ms();
  std::tuple<uint8_t, uint8_t> unpack_ottrim_values(uint8_t ottrim);
  void set_stall_detection_activation_level(float level) { this->sdal_ = level; };

  void add_on_alert_callback(std::function<void(DriverEvent)> &&callback) {
    this->on_alert_callback_.add(std::move(callback));
  }

 protected:
  const uint32_t clk_frequency_;

  InternalGPIOPin *enn_pin_{nullptr};
  InternalGPIOPin *diag_pin_{nullptr};
  InternalGPIOPin *index_pin_{nullptr};
  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};

  bool is_enabled_;
  float sdal_{0.5};  // stall detection activation level. A percentage of max_speed.

  CallbackManager<void(const DriverEvent &event)> on_alert_callback_;

#if defined(ENABLE_DRIVER_ALERT_EVENTS)
  void check_driver_status_();

  EventHandler diag_handler_{};         // Event on DIAG
  EventHandler stalled_handler_{true};  // Stalled. Initial state is true so that first startup doesn't trigger
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
  EventHandler uvcp_handler_{};   // Charge pump undervoltage
#endif

#if defined(USE_DIAG_PIN)
  ISRPinTriggerStore diag_isr_store_{};
  bool diag_triggered_{false};
#endif

#if defined(USE_INDEX_PIN)
  ISRPinTriggerStore index_isr_store_{};
  bool index_triggered_{false};
#endif
};

}  // namespace tmc2209
}  // namespace esphome
