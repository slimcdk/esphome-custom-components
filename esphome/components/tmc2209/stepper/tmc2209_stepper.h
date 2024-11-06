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
namespace tmc2209_stepper {

using namespace tmc2209;
using namespace stepper;

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
  void stop() override;
  void enable(bool enable) override;
  void set_target(int32_t steps) override;
  bool is_stalled() override;

 protected:
  HighFrequencyLoopRequester high_freq_;

#if defined(SERIAL_CONTROL)
  IndexPulseStore ips_{};  // index pulse store
#endif

#if defined(PULSES_CONTROL)
  bool step_state_{false};
#endif
};

}  // namespace tmc2209_stepper
}  // namespace esphome
