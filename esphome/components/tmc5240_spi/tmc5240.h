#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/spi/spi.h"
#include "esphome/components/tmc5240/tmc5240.h"

namespace esphome {
namespace tmc5240_spi {

class TMC5240SPIStepper : public tmc5240::TMC5240Stepper,
                          public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH,
                                                spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_10MHZ> {
 public:
  void setup() override;

  TMC5240BusType get_bus_type() const override { return IC_BUS_SPI; }
};

}  // namespace tmc5240_spi
}  // namespace esphome
