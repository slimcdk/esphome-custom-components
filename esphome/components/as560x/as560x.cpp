#include "as560x.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace as560x {

static const char *TAG = "as560X";

void AS560X::set_zero_position(uint16_t position) {
  if (!this->write_byte_16(AS560X_REGISTER_ZPOS, position))
    return;
  this->zero_position_ = position;
};

bool AS560X::is_device_online() {
  auto ack = this->read_byte(AS560X_REGISTER_STATUS);
  return ack ? true : false;
}

int16_t AS560X::read_status_byte() {
  auto status = this->read_byte(AS560X_REGISTER_STATUS);
  return (int16_t) (status ? status.value() : -1);
}

bool AS560X::get_presence() {
  auto data = this->read_byte(AS560X_REGISTER_STATUS);
  if (!data)
    return false;
  return bitRead(data.value(), 5) == 1 ? true : false;
}

uint16_t AS560X::get_magnitude() {
  uint16_t data;
  if (!this->read_byte_16(AS560X_REGISTER_MAGNITUDE, &data))
    return this->magnitude_sensor_->get_raw_state();
  return data;
}

uint16_t AS560X::get_raw_angle() {
  uint16_t data;
  if (!this->read_byte_16(AS560X_REGISTER_RAWANGLE, &data))
    return 0;
  return data;
}

uint16_t AS560X::get_angle() {
  uint16_t data;
  if (!this->read_byte_16(AS560X_REGISTER_ANGLE, &data))
    return this->angle_sensor_->get_raw_state();
  return data;
}

uint8_t AS560X::get_gain() {
  auto data = this->read_byte(AS560X_REGISTER_AGC);
  if (!data)
    return 0;
  return data.value();
}

void AS560X::set_stop_position(uint16_t position) { this->stop_position_ = position; };

void AS560X::set_maximum_angle(uint16_t angle) { this->maximum_angle_ = angle; };

}  // namespace as560x
}  // namespace esphome
