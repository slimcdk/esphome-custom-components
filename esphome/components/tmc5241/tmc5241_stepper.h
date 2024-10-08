// file: esphome/components/tmc5241/tmc5241_stepper.h
/**
 * Communication agnostic TMC5241 stepper component.
 * Contains the logic for a ESPHome stepper.
 */

#pragma once

#include "tmc5241.h"

#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace tmc5241 {

class TMC5241Stepper : public TMC5241, public stepper::Stepper {
 public:
  TMC5241Stepper() = default;

  // void setup() override;
  // void loop() override;
  // void dump_config() override;
};

}  // namespace tmc5241
}  // namespace esphome
