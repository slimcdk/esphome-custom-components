#include "as5600.h"
#include "esphome/core/log.h"

namespace esphome {
namespace as560x {

static const char *TAG = "as5600";

void AS5600::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up device...");
  LOG_SENSOR("  ", "Angle Sensor", this->angle_sensor_);
  LOG_SENSOR("  ", "Magnitude Sensor", this->magnitude_sensor_);
  LOG_BINARY_SENSOR("  ", "Magnet Presence Binary Sensor", this->presence_binary_sensor_);
};

// void AS5600::set_zero_position(uint16_t position) { this->zero_position_ = position; };

}  // namespace as560x
}  // namespace esphome
