#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/tmc5240/tmc5240.h"

namespace esphome {
namespace tmc5240_uart {

class TMC5240UARTStepper : public tmc5240::TMC5240Stepper, public uart::UARTDevice {
 public:
  TMC5240UARTStepper(uint8_t address);
  void setup() override;

  TMC5240BusType get_bus_type() const override { return IC_BUS_UART; }

  uint8_t get_address() { return this->address_; }

 protected:
  uint8_t address_{0x00};
};

}  // namespace tmc5240_uart
}  // namespace esphome
