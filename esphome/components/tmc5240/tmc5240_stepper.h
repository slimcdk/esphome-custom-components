// file: esphome/components/tmc5240/tmc5240_stepper.h
/**
 * Communication agnostic TMC5240 stepper component.
 * Contains the logic for a ESPHome stepper.
 */

#pragma once

#include "tmc5240.h"

#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace tmc5240 {

class TMC5240Stepper : public TMC5240, public stepper::Stepper {
 public:
  TMC5240Stepper() = default;

  // void setup() override;
  // void loop() override;
  // void dump_config() override;
};

}  // namespace tmc5240
}  // namespace esphome
