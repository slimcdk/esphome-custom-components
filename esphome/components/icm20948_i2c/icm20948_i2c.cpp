
#include "esphome/core/log.h"
#include "icm20948_i2c.h"

namespace esphome {
namespace icm20948_i2c {

static const char *TAG = "icm20948.i2c.sensor";

void ICM20948I2C::dump_config() {
  LOG_ICM20948(this);
  LOG_I2C_DEVICE(this);
}

bool ICM20948I2C::write_8(uint16_t reg, uint8_t value) { return true; }
bool ICM20948I2C::write_16(uint16_t reg, uint16_t value) { return true; }
bool ICM20948I2C::write_32(uint16_t reg, uint32_t value) { return true; }
bool ICM20948I2C::read_8(uint16_t reg, uint8_t *value) { return true; }
bool ICM20948I2C::read_16(uint16_t reg, uint16_t *value) { return true; }
bool ICM20948I2C::read_32(uint16_t reg, uint32_t *value) { return true; }

}  // namespace icm20948_i2c
}  // namespace esphome
