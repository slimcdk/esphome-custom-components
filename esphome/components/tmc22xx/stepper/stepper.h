#pragma once
#include "esphome/components/tmc22xx/tmc22xx_registers.h"
#include "esphome/components/tmc22xx/tmc22xx_api.h"
#include "esphome/components/tmc22xx/tmc22xx_component.h"
#include "esphome/components/tmc22xx/tmc22xx_events.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace tmc22xx {

using namespace esphome::stepper;

enum ControlMethod {
  CONTROL_UNSET,
  SERIAL_CONTROL,
  PULSES_CONTROL,
};

struct IndexPulseStore {
  int32_t *current_position_ptr{nullptr};
  Direction *direction_ptr{nullptr};
  static void IRAM_ATTR HOT pulse_isr(IndexPulseStore *arg) {
    (*(arg->current_position_ptr)) += (int8_t) (*(arg->direction_ptr));
  }
};

class TMC22XXStepper : public TMC22XXComponent, public Stepper {
 public:
  TMC22XXStepper() = default;
  TMC22XXStepper(uint8_t address) : TMC22XXComponent(address){};

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

/*
class TMC2202Stepper : public TMC22XXStepper {
 public:
  TMC2202Stepper() = default;
  TMC2202Stepper(uint8_t address) : TMC22XXStepper(address){};
};

class TMC2208Stepper : public TMC22XXStepper {
 public:
  TMC2208Stepper() = default;
  TMC2208Stepper(uint8_t address) : TMC22XXStepper(address){};
};

class TMC2209Stepper : public TMC22XXStepper {
 public:
  TMC2209Stepper() = default;
  TMC2209Stepper(uint8_t address) : TMC22XXStepper(address){};
};

class TMC2224Stepper : public TMC22XXStepper {
 public:
  TMC2224Stepper() = default;
  TMC2224Stepper(uint8_t address) : TMC22XXStepper(address){};
};

class TMC2225Stepper : public TMC22XXStepper {
 public:
  TMC2225Stepper() = default;
  TMC2225Stepper(uint8_t address) : TMC22XXStepper(address){};
};

class TMC2226Stepper : public TMC22XXStepper {
 public:
  TMC2226Stepper() = default;
  TMC2226Stepper(uint8_t address) : TMC22XXStepper(address){};
};
*/

}  // namespace tmc22xx
}  // namespace esphome
