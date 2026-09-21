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

using namespace esphome::stepper;

enum ControlMethod {
  CONTROL_UNSET,
  SERIAL_CONTROL,
  PULSES_CONTROL,
};

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  // Number of INDEX edges per reported step. Needed because INDEX_STEP
  // toggles per internal (interpolated) microstep, while ESPHome steps
  // are in MRES units. E.g. microsteps=8 with interpolation to 256
  // produces 32 edges per step and needs divider=32. Default 1 preserves
  // the old microsteps=1 behaviour.
  volatile uint16_t divider{1};
  volatile uint16_t counter{0};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) {
    if (++arg->counter >= arg->divider) {
      arg->counter = 0;
      (*(arg->current_position_ptr)) += (int8_t) (*(arg->direction_ptr));
    }
  }
};

class TMC2209Stepper : public TMC2209Component, public Stepper {
 public:
  TMC2209Stepper() = default;
  TMC2209Stepper(uint8_t address) : TMC2209Component(address){};

  void dump_config() override;
  void setup() override;
  void loop() override;
  void on_shutdown() override;
  void stop() override;
  void enable(bool enable) override;
  // void enable(bool enable, bool recover_toff = true) override;
  void set_target(int32_t steps) override;
  bool is_stalled() override;

  void set_control_method(ControlMethod method) { this->control_method_ = method; }
  void set_index_divider(uint16_t divider) override {
    if (divider < 1)
      divider = 1;
    this->ips_.divider = divider;
    this->ips_.counter = 0;
  }

 protected:
  HighFrequencyLoopRequester high_freq_;
  ControlMethod control_method_{ControlMethod::CONTROL_UNSET};

  /** Serial control */
  IndexPulseStore ips_;  // index pulse store
  int32_t vactual_ = 0;
  /* */

  /** Pulses control */
  bool step_state_ = false;
  Direction direction_;
  /* */
};

}  // namespace tmc2209
}  // namespace esphome
