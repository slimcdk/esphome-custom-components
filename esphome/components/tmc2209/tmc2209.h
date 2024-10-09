#pragma once

#include <limits>
#include <string>
#include <tuple>
#include <iostream>
// #include <stdint.h>
// #include <stdbool.h>
// #include <stddef.h>

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/stepper/stepper.h"
#include "tmc2209_registers.h"

namespace esphome {
namespace tmc2209 {

static const char *TAG = "tmc2209";

#define ACCESS_READ 0x01
#define IS_READABLE(x) ((x) &ACCESS_READ)

// Default Register values
#define R00 ((int32_t) 0x00000040)  // GCONF
#define R10 ((int32_t) 0x00071703)  // IHOLD_IRUN
#define R11 ((int32_t) 0x00000014)  // TPOWERDOWN
#define R6C ((int32_t) 0x10000053)  // CHOPCONF
#define R70 ((int32_t) 0xC10D0024)  // PWMCONF

#define ____ 0x00

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

class TMC2209 : public Component, public stepper::Stepper, public uart::UARTDevice {
 public:
  TMC2209(uint8_t address, uint32_t clk_frequency) : address_(address), clk_frequency_(clk_frequency){};

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps);
  void stop() override;

  void enable(bool enable);

  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; };
  void set_enn_pin(GPIOPin *pin) { this->enn_pin_ = pin; };
  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; };
  void set_dir_pin(GPIOPin *pin) { this->dir_pin_ = pin; };
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };

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

  // Write or read a register (all fields) or register field (single field within register)
  void write_register(uint8_t address, int32_t value);
  int32_t read_register(uint8_t address);
  void write_field(RegisterField field, uint32_t value);
  uint32_t read_field(RegisterField field);
  uint32_t extract_field(uint32_t data, RegisterField field);
  uint32_t update_field(uint32_t data, RegisterField field, uint32_t value);

 protected:
  const uint8_t address_;
  const uint32_t clk_frequency_;
  uint8_t dirty_bits_[REGISTER_COUNT / 8] = {0};
  int32_t shadow_register_[REGISTER_COUNT];

  // Register access permissions:
  //   0x00: none (reserved)
  //   0x01: read
  //   0x02: write
  //   0x03: read/write
  //   0x23: read/write, flag register (write to clear)
  const uint8_t register_access_[REGISTER_COUNT] = {
      //  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
      0x03, 0x23, 0x01, 0x02, 0x02, 0x01, 0x01, 0x03, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x00 - 0x0F
      0x02, 0x02, 0x01, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x10 - 0x1F
      ____, ____, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x20 - 0x2F
      ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x30 - 0x3F
      0x02, 0x01, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x40 - 0x4F
      ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x50 - 0x5F
      ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, 0x01, 0x01, 0x03, ____, ____, 0x01,  // 0x60 - 0x6F
      0x03, 0x01, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____   // 0x70 - 0x7F
  };

  const uint8_t tmcCRCTable_Poly7Reflected[256] = {
      0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, 0x1C, 0x8D, 0xFF,
      0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE,
      0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, 0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A,
      0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, 0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C,
      0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86,
      0x17, 0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 0x54, 0xC5,
      0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 0xE0, 0x71, 0x03, 0x92, 0xE7,
      0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
      0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35,
      0xA4, 0xD1, 0x40, 0x32, 0xA3, 0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C,
      0x2E, 0xBF, 0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 0x8C,
      0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8, 0x39, 0x4B, 0xDA,
      0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, 0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50,
      0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
  };

  void set_dirty_bit_(uint8_t index, bool value);
  bool get_dirty_bit_(uint8_t index);
  bool cache_(CacheOp operation, uint8_t address, uint32_t *value);
  bool read_write_register_(uint8_t *data, size_t writeLength, size_t readLength);
  uint8_t crc8(uint8_t *data, uint32_t bytes);

  InternalGPIOPin *diag_pin_{nullptr};
  InternalGPIOPin *index_pin_{nullptr};
  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  GPIOPin *enn_pin_{nullptr};
  float sdal_{0.5};  // stall detection activation level. A percentage of max_speed.
  bool is_enabled_;

#if defined(USE_DIAG_PIN)
  ISRPinTriggerStore diag_isr_store_{};
  bool diag_isr_triggered_{false};
#endif

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

#if defined(USE_UART_CONTROL)
  IndexPulseStore ips_{};  // index pulse store
#endif

  HighFrequencyLoopRequester high_freq_;
  Direction direction_{Direction::NONE};
};

}  // namespace tmc2209
}  // namespace esphome
