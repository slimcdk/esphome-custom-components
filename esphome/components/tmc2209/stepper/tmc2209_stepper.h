#pragma once

#include <limits>
#include <string>

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/tmc2209/tmc2209.h"

namespace esphome {
namespace tmc2209 {

enum Direction : int8_t {
  CLOCKWISE = -1,     // Moving one direction
  NONE = 0,           // Not moving
  ANTICLOCKWISE = 1,  // Moving the other direction
};

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  int32_t *target_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) { (*(arg->current_position_ptr)) += *(arg->direction_ptr); }
};

class TMC2209Stepper : public Parented<TMC2209>, public Component, public stepper::Stepper {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void stop() override;

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; };
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; };

  void enable(bool enable = true);

 protected:
  InternalGPIOPin *enn_pin_;
  InternalGPIOPin *index_pin_;

  bool driver_is_enabled_{false};
  IndexPulseStore ips_{};  // index pulse store
  Direction direction_{Direction::NONE};
  HighFrequencyLoopRequester high_freq_;

  void run_driver_activation_();

  bool prev_has_reached_target_{this->has_reached_target()};
  int32_t prev_current_position_{this->current_position};
};

}  // namespace tmc2209
}  // namespace esphome
