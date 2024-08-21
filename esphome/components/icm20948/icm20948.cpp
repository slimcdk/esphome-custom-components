
#include "esphome/core/log.h"
#include "icm20948.h"

namespace esphome {
namespace icm20948 {

static const char *TAG = "icm20948.sensor";

void ICM20948::dump_config() {
  ESP_LOGCONFIG(TAG, "ICM20948:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with ICM20948 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void ICM20948::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ICM20948...");

  uint8_t deviceId;
  if (!this->read_byte(ICM20X_B0_WHOAMI, &deviceId)) {
    this->mark_failed();
    return;
  }

  if (deviceId != ICM20948_CHIP_ID) {
    ESP_LOGE(TAG, "Expected device id to be 0x%X but got 0x%X", ICM20948_CHIP_ID, deviceId);
    this->mark_failed();
    return;
  } else {
    ESP_LOGV(TAG, "Connected device has id 0x%X", deviceId);
  }
}

void ICM20948::update() {
  uint16_t temp;

  if (!this->read_byte_16(0x3A, &temp)) {
    this->mark_failed();
  }

  ESP_LOGD(TAG, "temp: %d", temp);

  /*
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(gyro_z);
      */
}

}  // namespace icm20948
}  // namespace esphome
