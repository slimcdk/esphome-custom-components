#pragma once

#include <limits>
#include <string>
#include <tuple>
#include <iostream>

#include "tmc2209.h"
#include "tmc2209_api.h"
#include "tmc2209_api_registers.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace tmc2209 {

static const char *TAG = "tmc2209.stepper";

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  stepper::Direction *direction_ptr{nullptr};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) { (*(arg->current_position_ptr)) += *(arg->direction_ptr); }
};

class TMC2209Stepper : public TMC2209, public Component, public stepper::Stepper {
 public:
  TMC2209Stepper(uint8_t address, uint32_t clk_frequency) : TMC2209(address, clk_frequency){};

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps);
  void stop() override;

  void enable(bool enable);

 protected:
#if defined(USE_UART_CONTROL)
  IndexPulseStore ips_{};  // index pulse store
#endif

  HighFrequencyLoopRequester high_freq_;
};

}  // namespace tmc2209
}  // namespace esphome
