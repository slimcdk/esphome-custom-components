#include "as5601.h"
#include "esphome/core/log.h"

namespace esphome {
namespace as560x {

#define AS5601_REGISTER_ABN 0x09
#define AS5601_REGISTER_PUSHTHR 0x0A

static const char *TAG = "as5601";

void AS5601::dump_config() {
  ESP_LOGCONFIG(TAG, "AS5601:");
  LOG_SENSOR("  ", "Angle Sensor", this->angle_sensor_);
  LOG_SENSOR("  ", "Magnitude Sensor", this->magnitude_sensor_);
  LOG_BINARY_SENSOR("  ", "Magnet Presence Binary Sensor", this->presence_binary_sensor_);
};

void AS5601::loop() {
  if (this->presence_binary_sensor_ != nullptr) {
    this->presence_binary_sensor_->publish_state(this->get_presence());
  }

  if (this->magnitude_sensor_ != nullptr) {
    uint data = get_magnitude();
    this->magnitude_sensor_->publish_state(data);
  }

  if (this->angle_sensor_ != nullptr) {
    const uint16_t data = get_angle();
    const float angle = remap<float, uint16_t>(data, 0, 4096, 0, 360);
    this->angle_sensor_->publish_state(angle);
  }
}

void AS5601::set_ab_resolution(uint16_t positions) {
  // from https://github.com/bitfasching/AS5601/blob/master/AS5601.h
  uint8_t power = -1;
  while ((1 << ++power) < this->ab_resolution_) {
  }
  if (!this->write_byte(AS5601_REGISTER_ABN, power - 3)) {
    return;
  }
  this->ab_resolution_ = positions;
};

void AS5601::set_push_threshold(uint16_t value){};

}  // namespace as560x
}  // namespace esphome
