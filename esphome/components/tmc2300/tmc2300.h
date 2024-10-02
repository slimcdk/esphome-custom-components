#pragma once

#include <limits>
#include <string>
#include <tuple>
#include <iostream>

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/stepper/stepper.h"

extern "C" {
#include <ic/TMC2300/TMC2300.h>
}

namespace esphome {
namespace tmc2300 {

static const char *TAG = "tmc2300";

#define TMC2300_IC_VERSION_64 0x40

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

class TMC2300;  // forwared declare

static TMC2300 *components[TMC2300_NUM_COMPONENTS];
static uint16_t component_index = 0;

enum Direction : int8_t {
  CLOCKWISE = -1,     // Moving one direction
  NONE = 0,           // Not moving
  ANTICLOCKWISE = 1,  // Moving the other direction
};

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) { (*(arg->current_position_ptr)) += *(arg->direction_ptr); }
};

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

class TMC2300 : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2300(uint8_t address, uint32_t clock_frequency);

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps);
  void stop() override;

  void enable(bool enable);
  bool is_enabled() { return !this->en_pin_state_; };

  void set_en_pin(GPIOPin *pin) { this->en_pin_ = pin; };
  void set_nstdby_pin(GPIOPin *pin) { this->nstdby_pin_ = pin; };
  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; };
  void set_dir_pin(GPIOPin *pin) { this->dir_pin_ = pin; };
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };

  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();

  float get_motor_load();
  bool is_stalled();

  /* run and hold currents */
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

  void set_tpowerdown_ms(uint32_t delay_in_ms);
  uint32_t get_tpowerdown_ms();

  void add_on_alert_callback(std::function<void(DriverEvent)> &&callback) {
    this->on_alert_callback_.add(std::move(callback));
  }

  // TMC-API wrappers
  uint8_t get_address() { return this->address_; }

  // Write or read a register (all fields) or register field (single field within register)
  void write_register(uint8_t address, int32_t value);
  int32_t read_register(uint8_t address);
  void write_field(RegisterField field, uint32_t value);
  uint32_t read_field(RegisterField field);

 protected:
  uint16_t id_;  // used for tmcapi id index and esphome global component index
  uint8_t address_;
  const uint32_t clock_frequency_;

  GPIOPin *en_pin_{nullptr};
  GPIOPin *nstdby_pin_{nullptr};
  InternalGPIOPin *diag_pin_{nullptr};
  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  bool en_pin_state_;

  IndexPulseStore ips_{};  // index pulse store

#if defined(ENABLE_DRIVER_ALERT_EVENTS)
  void check_driver_status_();

  bool monitor_stallguard_{false};

  EventHandler diag_handler_{};     // Event on DIAG
  EventHandler stalled_handler_{};  // Stalled
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
  CallbackManager<void(const DriverEvent &event)> on_alert_callback_;

  HighFrequencyLoopRequester high_freq_;
  Direction direction_{Direction::NONE};
};

}  // namespace tmc2300
}  // namespace esphome
