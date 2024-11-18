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
  bool write_8(uint16_t reg, uint8_t value) override;
  bool write_16(uint16_t reg, uint16_t value) override;
  bool write_32(uint16_t reg, uint32_t value) override;
  bool read_8(uint16_t reg, uint8_t *value) override;
  bool read_16(uint16_t reg, uint16_t *value) override;
  bool read_32(uint16_t reg, uint32_t *value) override;
};

}  // namespace icm20948_i2c
}  // namespace esphome
