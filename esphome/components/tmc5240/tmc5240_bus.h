// #pragma once

// #include "esphome/core/helpers.h"
// #include "esphome/core/component.h"
// #include "esphome/core/hal.h"

// #include "esphome/components/stepper/stepper.h"
// #include "esphome/components/spi/spi.h"
// #include "esphome/components/uart/uart.h"

// extern "C" {
// #include <ic/TMC5240/TMC5240.h>
// #include <helpers/Bits.h>
// #include <helpers/Macros.h>
// }

// namespace esphome {
// namespace tmc5240 {

// #if defined TMC5240_USE_SPI
// class TMC5240Bus : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH,
// spi::CLOCK_PHASE_TRAILING,
//                                          spi::DATA_RATE_10MHZ> {
//   void setup() { this->setup_spi(); };
//   TMC5240BusType get_bus_type() const { return IC_BUS_SPI; }

// }
// #endif

// #if defined TMC5240_USE_UART
// class TMC5240Bus : public uart::UARTDevice {
//  public:
//   TMC5240Bus(uint8_t address) : address_(address) {}
//   void setup();
//   TMC5240BusType get_bus_type() const { return IC_BUS_UART; }

//   uint8_t get_address() { return this->address_; }

//  protected:
//   uint8_t address_{0x00};
// };
// #endif

// }  // namespace tmc5240
// }  // namespace esphome
