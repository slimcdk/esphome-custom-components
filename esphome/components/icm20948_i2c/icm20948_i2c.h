#pragma once

#include "esphome/core/helpers.h"
#include "esphome/components/icm20948/icm20948.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace icm20948_i2c {

class ICM20948I2C : public icm20948::ICM20948, public i2c::I2CDevice {
 public:
  void dump_config() override;

 protected:
  bool write(uint8_t reg, uint8_t data) override;
  bool read(uint8_t reg, uint8_t *data) override;
};

}  // namespace icm20948_i2c
}  // namespace esphome
