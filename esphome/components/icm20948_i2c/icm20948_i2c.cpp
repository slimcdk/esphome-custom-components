
#include "icm20948_i2c.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace icm20948_i2c {

static const char *TAG = "icm20948.i2c.sensor";

void ICM20948I2C::dump_config() {
  LOG_ICM20948(this);
  LOG_I2C_DEVICE(this);
}

bool ICM20948I2C::write(uint8_t reg, uint8_t *data, size_t len) {
  const i2c::ErrorCode ret = this->write_register(reg, data, len);
  return (ret == i2c::ErrorCode::NO_ERROR || ret == i2c::ErrorCode::ERROR_OK);
}

bool ICM20948I2C::read(uint8_t reg, uint8_t *data, size_t len) {
  const i2c::ErrorCode ret = this->read_register(reg, data, len);
  return (ret == i2c::ErrorCode::NO_ERROR || ret == i2c::ErrorCode::ERROR_OK);
}

}  // namespace icm20948_i2c
}  // namespace esphome
