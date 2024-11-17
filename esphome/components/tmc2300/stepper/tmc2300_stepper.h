#pragma once
#include "esphome/components/tmc2300/tmc2300_api_registers.h"
#include "esphome/components/tmc2300/tmc2300_api.h"
#include "esphome/components/tmc2300/tmc2300_component.h"
#include "esphome/components/tmc2300/events.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace tmc2300_stepper {

using namespace tmc2300;
using namespace stepper;

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) {
    (*(arg->current_position_ptr)) += (int8_t) (*(arg->direction_ptr));
  }
};

class TMC2300Stepper : public TMC2300Component, public Stepper {
 public:
  TMC2300Stepper(uint8_t address, uint32_t clk_frequency, bool internal_rsense, float rsense, bool analog_scale)
      : TMC2300Component(address, clk_frequency, internal_rsense, rsense, analog_scale){};

  void dump_config() override;
  void setup() override;
  void loop() override;
  void stop() override;
  void enable(bool enable) override;
  void set_target(int32_t steps) override;
  bool is_stalled() override;

 protected:
  HighFrequencyLoopRequester high_freq_;

#if defined(SERIAL_CONTROL)
  IndexPulseStore ips_;  // index pulse store
  int32_t vactual_ = 0;
#endif

#if defined(PULSES_CONTROL)
  bool step_state_ = false;
  Direction direction_;
#endif
};

}  // namespace tmc2300_stepper
}  // namespace esphome
