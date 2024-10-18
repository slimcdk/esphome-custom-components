#pragma once

#include "esphome/components/tmc5240/tmc5240_stepper.h"
#include "esphome/components/uart/uart.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tmc5240 {

class TMC5240UARTStepper : public TMC5240Stepper, public uart::UARTDevice {
 public:
  TMC5240UARTStepper() = default;

  void write_register(uint8_t address, int32_t value) override;
  int32_t read_register(uint8_t address) override;
};

}  // namespace tmc5240
}  // namespace esphome
