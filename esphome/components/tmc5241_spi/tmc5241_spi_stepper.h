// file: esphome/components/tmc5241_spi/tmc5241_spi.h
/**
 * SPI communication setup for TMC5241.
 * Provides access to the SPI interface.
 */

#pragma once

#include "esphome/components/tmc5241/tmc5241_stepper.h"
#include "esphome/components/spi/spi.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tmc5241 {

class TMC5241SPIStepper : public TMC5241Stepper,
                          public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH,
                                                spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_10MHZ> {
 public:
  TMC5241SPIStepper() = default;

  void setup() override { this->spi_setup(); }

  void write_register(uint8_t address, int32_t value) override;
  int32_t read_register(uint8_t address) override;
};

}  // namespace tmc5241
}  // namespace esphome
