#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace icm20948 {

#define LOG_ICM20948(this) \
  ESP_LOGCONFIG(TAG, "ICM20948:"); \
  LOG_UPDATE_INTERVAL(this); \
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_); \
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_); \
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_); \
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_); \
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_); \
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_); \
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_); \
  const uint8_t icid = this->read_icid(); \
  ESP_LOGCONFIG(TAG, "  Read IC ID: 0x%X", icid); \
  if (icid != ICM20948_CHIP_ID) \
    ESP_LOGE(TAG, "  Expected ID 0x%X", ICM20948_CHIP_ID);

#define ICM20948_CHIP_ID 0xEA  // <- ICM20948 default device id from WHOAMI

// Bank 0
#define ICM20X_B0_WHOAMI 0x00            // <- Chip ID register
#define ICM20X_B0_USER_CTRL 0x03         // <- User Control Reg. Includes I2C Master
#define ICM20X_B0_LP_CONFIG 0x05         // <- Low Power config
#define ICM20X_B0_REG_INT_PIN_CFG 0xF    // <- Interrupt config register
#define ICM20X_B0_REG_INT_ENABLE 0x10    // <- Interrupt enable register 0
#define ICM20X_B0_REG_INT_ENABLE_1 0x11  // <- Interrupt enable register 1
#define ICM20X_B0_I2C_MST_STATUS 0x17    // <- Records if I2C master bus data is finished
#define ICM20X_B0_REG_BANK_SEL 0x7F      // <- register bank selection register
#define ICM20X_B0_PWR_MGMT_1 0x06        // <- primary power management register
#define ICM20X_B0_ACCEL_XOUT_H 0x2D      // <- first byte of accel data
#define ICM20X_B0_GYRO_XOUT_H 0x33       // <- first byte of accel data

// Bank 2
#define ICM20X_B2_GYRO_SMPLRT_DIV 0x00     // <- Gyroscope data rate divisor
#define ICM20X_B2_GYRO_CONFIG_1 0x01       // <- Gyro config for range setting
#define ICM20X_B2_ACCEL_SMPLRT_DIV_1 0x10  // <- Accel data rate divisor MSByte
#define ICM20X_B2_ACCEL_SMPLRT_DIV_2 0x11  // <- Accel data rate divisor LSByte
#define ICM20X_B2_ACCEL_CONFIG_1 0x14      // <- Accel config for setting range

// Bank 3
#define ICM20X_B3_I2C_MST_ODR_CONFIG 0x0  // <- Sets ODR for I2C master bus
#define ICM20X_B3_I2C_MST_CTRL 0x1        // <- I2C master bus config
#define ICM20X_B3_I2C_MST_DELAY_CTRL 0x2  // <- I2C master bus config
#define ICM20X_B3_I2C_SLV0_ADDR 0x3       // <- Sets I2C address for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_REG 0x4        // <- Sets register address for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_CTRL 0x5       // <- Controls for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_DO 0x6         // <- Sets I2C master bus slave 0 data out

#define ICM20X_B3_I2C_SLV4_ADDR 0x13  // <- Sets I2C address for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_REG 0x14   // <- Sets register address for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_CTRL 0x15  // <- Controls for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_DO 0x16    // <- Sets I2C master bus slave 4 data out
#define ICM20X_B3_I2C_SLV4_DI 0x17    // <- Sets I2C master bus slave 4 data in

#define ICM20948_MAG_ID 0x09  // <- The chip ID for the magnetometer

#define ICM20948_UT_PER_LSB 0.15  // <- mag data LSB value (fixed)

#define AK09916_WIA2 0x01   // <- Magnetometer
#define AK09916_ST1 0x10    // <- Magnetometer
#define AK09916_HXL 0x11    // <- Magnetometer
#define AK09916_HXH 0x12    // <- Magnetometer
#define AK09916_HYL 0x13    // <- Magnetometer
#define AK09916_HYH 0x14    // <- Magnetometer
#define AK09916_HZL 0x15    // <- Magnetometer
#define AK09916_HZH 0x16    // <- Magnetometer
#define AK09916_ST2 0x18    // <- Magnetometer
#define AK09916_CNTL2 0x31  // <- Magnetometer
#define AK09916_CNTL3 0x32  // <- Magnetometer

/** The accelerometer data range */
typedef enum {
  ICM20948_ACCEL_RANGE_2_G,
  ICM20948_ACCEL_RANGE_4_G,
  ICM20948_ACCEL_RANGE_8_G,
  ICM20948_ACCEL_RANGE_16_G,
} icm20948_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20948_GYRO_RANGE_250_DPS,
  ICM20948_GYRO_RANGE_500_DPS,
  ICM20948_GYRO_RANGE_1000_DPS,
  ICM20948_GYRO_RANGE_2000_DPS,
} icm20948_gyro_range_t;

/**
 * @brief Data rates/modes for the embedded AsahiKASEI AK09916 3-axis
 * magnetometer
 *
 */
typedef enum {
  AK09916_MAG_DATARATE_SHUTDOWN = 0x0,  // <- Stops measurement updates
  AK09916_MAG_DATARATE_SINGLE = 0x1,    // <- Takes a single measurement then switches to
                                        // <- AK09916_MAG_DATARATE_SHUTDOWN
  AK09916_MAG_DATARATE_10_HZ = 0x2,     // <- updates at 10Hz
  AK09916_MAG_DATARATE_20_HZ = 0x4,     // <- updates at 20Hz
  AK09916_MAG_DATARATE_50_HZ = 0x6,     // <- updates at 50Hz
  AK09916_MAG_DATARATE_100_HZ = 0x8,    // <- updates at 100Hz
} ak09916_data_rate_t;

class ICM20948 : public PollingComponent, public sensor::Sensor {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  // void setup() override;
  void update() override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

  uint8_t read_b0();

  uint8_t read_icid();

 protected:
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  virtual bool write(uint8_t reg, uint8_t *data, size_t len = 8) = 0;
  virtual bool read(uint8_t reg, uint8_t *data, size_t len = 8) = 0;
};

}  // namespace icm20948
}  // namespace esphome
