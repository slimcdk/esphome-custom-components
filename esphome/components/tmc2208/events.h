#pragma once
#include "tmc2208_api_registers.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2208 {

enum DriverStatusEvent {
  DIAG_TRIGGERED = 0,
  DIAG_TRIGGER_CLEARED = 1,

  RESET = 2,
  RESET_CLEARED = 3,
  DRIVER_ERROR = 4,
  DRIVER_ERROR_CLEARED = 5,
  CP_UNDERVOLTAGE = 6,
  CP_UNDERVOLTAGE_CLEARED = 7,

  // TEMPERATURE_NORMAL = 10,
  OVERTEMPERATURE_PREWARNING = 11,
  OVERTEMPERATURE_PREWARNING_CLEARED = 12,
  OVERTEMPERATURE = 13,
  OVERTEMPERATURE_CLEARED = 14,
  TEMPERATURE_ABOVE_120C = 15,
  TEMPERATURE_BELOW_120C = 16,
  TEMPERATURE_ABOVE_143C = 17,
  TEMPERATURE_BELOW_143C = 18,
  TEMPERATURE_ABOVE_150C = 19,
  TEMPERATURE_BELOW_150C = 20,
  TEMPERATURE_ABOVE_157C = 21,
  TEMPERATURE_BELOW_157C = 22,

  OPEN_LOAD = 30,
  OPEN_LOAD_CLEARED = 31,
  OPEN_LOAD_A = 32,
  OPEN_LOAD_A_CLEARED = 33,
  OPEN_LOAD_B = 34,
  OPEN_LOAD_B_CLEARED = 35,

  LOW_SIDE_SHORT = 40,
  LOW_SIDE_SHORT_CLEARED = 41,
  LOW_SIDE_SHORT_A = 42,
  LOW_SIDE_SHORT_A_CLEARED = 43,
  LOW_SIDE_SHORT_B = 44,
  LOW_SIDE_SHORT_B_CLEARED = 45,

  GROUND_SHORT = 50,
  GROUND_SHORT_CLEARED = 51,
  GROUND_SHORT_A = 52,
  GROUND_SHORT_A_CLEARED = 53,
  GROUND_SHORT_B = 54,
  GROUND_SHORT_B_CLEARED = 55,
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

}  // namespace tmc2208
}  // namespace esphome
