#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace as560x {

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define AS560X_REGISTER_ZMCO 0x00
#define AS560X_REGISTER_ZPOS 0x01
#define AS560X_REGISTER_CONF 0x07
#define AS560X_REGISTER_RAWANGLE 0x0C
#define AS560X_REGISTER_ANGLE 0x0E
#define AS560X_REGISTER_STATUS 0x0B
#define AS560X_REGISTER_AGC 0x1A
#define AS560X_REGISTER_MAGNITUDE 0x1B
#define AS560X_REGISTER_BURN 0xFF

class AS560X : public i2c::I2CDevice {
 public:
  AS560X() = default;

  void set_presence_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    this->presence_binary_sensor_ = binary_sensor;
  }
  void set_angle_sensor(sensor::Sensor *sensor) { this->angle_sensor_ = sensor; }
  void set_magnitude_sensor(sensor::Sensor *sensor) { this->magnitude_sensor_ = sensor; }

  void set_maximum_angle(uint16_t angle);
  void set_stop_position(uint16_t position);
  void set_zero_position(uint16_t position);

  uint16_t get_magnitude();
  uint16_t get_angle();
  uint16_t get_raw_angle();
  uint8_t get_gain();
  bool get_presence();

 protected:
  int16_t read_status_byte();
  bool is_device_online();

  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *magnitude_sensor_{nullptr};

  uint16_t zero_position_{0};
  uint16_t stop_position_{4095};
  uint16_t maximum_angle_{359};
};

}  // namespace as560x
}  // namespace esphome
