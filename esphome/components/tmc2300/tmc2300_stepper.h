#pragma once

#include "tmc2300_api_registers.h"

#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2300 {

class TMC2300Stepper : public TMC2300Component, public stepper::Stepper {
 public:
  TMC2300Stepper(uint8_t address, uint32_t clk_frequency, bool internal_sense, float rsense)
      : TMC2300Component(address, clk_frequency, internal_sense, rsense){};
};

}  // namespace tmc2300
}  // namespace esphome
