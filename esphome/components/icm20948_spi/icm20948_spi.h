#pragma once

#include "esphome/core/helpers.h"
#include "esphome/components/icm20948/icm20948.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace icm20948_spi {

class ICM20948SPI : public icm20948::ICM20948, public spi::SPIDevice {
 public:
  void setup() override;
  void on_shutdown() override;
  void dump_config() override;
};

}  // namespace icm20948_spi
}  // namespace esphome
