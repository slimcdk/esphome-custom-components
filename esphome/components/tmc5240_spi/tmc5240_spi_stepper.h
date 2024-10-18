// file: esphome/components/tmc5240_spi/tmc5240_spi.h
/**
 * SPI communication setup for TMC5240.
 * Provides access to the SPI interface.
 */

#pragma once

#include "esphome/components/tmc5240/tmc5240_stepper.h"
#include "esphome/components/spi/spi.h"

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tmc5240 {

class TMC5240SPIStepper : public TMC5240Stepper,
                          public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH,
                                                spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_10MHZ> {
 public:
  TMC5240SPIStepper() = default;

  void setup() override { this->spi_setup(); }

  void write_register(uint8_t address, int32_t value) override;
  int32_t read_register(uint8_t address) override;
};

}  // namespace tmc5240
}  // namespace esphome
