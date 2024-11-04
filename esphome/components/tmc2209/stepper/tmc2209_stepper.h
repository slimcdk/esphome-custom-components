#pragma once
#include "esphome/components/tmc2209/tmc2209_api_registers.h"
#include "esphome/components/tmc2209/tmc2209_api.h"
#include "esphome/components/tmc2209/tmc2209_component.h"
#include "esphome/components/tmc2209/events.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace tmc2209 {

using namespace stepper;

#define LOOP_SAMPLES 5

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) {
    (*(arg->current_position_ptr)) += (int8_t) (*(arg->direction_ptr));
  }
};

class TMC2209Stepper : public TMC2209Component, public Stepper {
 public:
  TMC2209Stepper(uint8_t address, uint32_t clk_frequency, bool internal_rsense, float rsense, bool analog_scale)
      : TMC2209Component(address, clk_frequency, internal_rsense, rsense, analog_scale){};

  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps);
  void stop() override;
  void enable(bool enable);

  void set_stall_detection_activation_level(float level) { this->sdal_ = level; };

  void add_on_stall_callback(std::function<void()> &&callback) { this->on_stall_callback_.add(std::move(callback)); }

 protected:
  HighFrequencyLoopRequester high_freq_;

  IndexPulseStore ips_{};  // index pulse store
  int32_t velocity_{0};

  float sdal_{0.5};  // stall detection activation level. A percentage of max_speed.
  EventHandler stall_handler_;
  CallbackManager<void()> on_stall_callback_;

  void serial_control_(time_t now);
  void pulses_control_(time_t now);

  /*
  time_t loop_time_(time_t now);
  time_t last_loop_{0};
  std::array<time_t, LOOP_SAMPLES> loop_deltas_{0};
  uint8_t loop_deltas_i_{0};

  time_t loop_time_(time_t now = micros()) {
    this->loop_deltas_.at(this->loop_deltas_i_++ % LOOP_SAMPLES) = (now - this->last_loop_);
    this->last_loop_ = now;

    time_t loop_delta_sum = 0;
    for (uint8_t i = 0; i < LOOP_SAMPLES; i++) {
      loop_delta_sum += this->loop_deltas_[i];
    }

    return loop_delta_sum / LOOP_SAMPLES;
  }

  */

  bool step_state_{false};
  // time_t last_step_{0};
};

}  // namespace tmc2209
}  // namespace esphome
