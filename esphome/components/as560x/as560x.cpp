#include "as560x.h"
#include "esphome/core/log.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

namespace esphome {
namespace as560x {

/** AS5600 and AS5601 Register Map **/
// Common
static const uint8_t REGISTER_ZMCO = 0x00;
static const uint16_t REGISTER_ZPOS = 0x01;
static const uint16_t REGISTER_CONF = 0x07;
static const uint16_t REGISTER_RAWANGLE = 0x0C;
static const uint16_t REGISTER_ANGLE = 0x0E;
static const uint8_t REGISTER_STATUS = 0x0B;
static const uint8_t REGISTER_AGC = 0x1A;
static const uint16_t REGISTER_MAGNITUDE = 0x1B;
static const uint8_t REGISTER_BURN = 0xFF;

// AS5600
// static const uint16_t REGISTER_MPOS = 0x03;
// static const uint16_t REGISTER_MANG = 0x05;

// AS5601
// static const uint8_t REGISTER_ABN = 0x09;
// static const uint8_t REGISTER_PUSHTHR = 0x0A;
/** **/

static const char *TAG = "as560X";

void AS560XComponent::setup() { common_setup(); }

void AS560XComponent::common_setup() {
  // Common device config
  this->write_zero_position(this->zero_position_);
}

void AS560XComponent::loop() {
  if (this->presence_binary_sensor_ != nullptr)
    this->presence_binary_sensor_->publish_state(presence());

  if (this->magnitude_sensor_ != nullptr) {
    uint data = magnitude();
    if (data != this->magnitude_sensor_->get_raw_state())
      this->magnitude_sensor_->publish_state(data);
  }

  if (this->angle_sensor_ != nullptr) {
    uint data = angle();
    if (data != this->angle_sensor_->get_raw_state())
      this->angle_sensor_->publish_state(data);
  }
}

void AS560XComponent::write_zero_position(uint16_t position) {
  if (!this->write_byte_16(REGISTER_ZPOS, position))
    return;
  this->zero_position_ = position;
};

bool AS560XComponent::is_device_online() {
  auto ack = this->read_byte(REGISTER_STATUS);
  return ack ? true : false;
}

int16_t AS560XComponent::read_status_byte() {
  auto status = this->read_byte(REGISTER_STATUS);
  return (int16_t) (status ? status.value() : -1);
}

bool AS560XComponent::presence() {
  auto data = this->read_byte(REGISTER_STATUS);
  if (!data)
    return false;
  return bitRead(data.value(), 5) == 1 ? true : false;
}

uint16_t AS560XComponent::magnitude() {
  uint16_t data;
  if (!this->read_byte_16(REGISTER_MAGNITUDE, &data))
    return this->magnitude_sensor_->get_raw_state();
  return data;
}

uint16_t AS560XComponent::raw_angle() {
  uint16_t data;
  if (!this->read_byte_16(REGISTER_RAWANGLE, &data))
    return 0;
  return data;
}

uint16_t AS560XComponent::angle() {
  uint16_t data;
  if (!this->read_byte_16(REGISTER_ANGLE, &data))
    return this->angle_sensor_->get_raw_state();
  return data;
}

uint8_t AS560XComponent::gain() {
  auto data = this->read_byte(REGISTER_AGC);
  if (!data)
    return 0;
  return data.value();
}

/** AS5600 Register Map **/
static const uint16_t REGISTER_MPOS = 0x03;
static const uint16_t REGISTER_MANG = 0x05;
/** **/

static const char *TAG = "as5600";

void AS5600Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up device...");
  ESP_LOGCONFIG(TAG, "  Angle Stop Position: %d", this->stop_position_);
  ESP_LOGCONFIG(TAG, "  Maximum Angle: %d", this->maximum_angle_);
  LOG_AS560X(this);
};

void AS5600Component::setup() {
  // if (this->is_device_online()) return this->mark_failed();
  AS560XComponent::common_setup();
  this->write_stop_position(this->stop_position_);
  this->write_maximum_angle(this->maximum_angle_);
};

void AS5600Component::write_stop_position(uint16_t position) {
  /* write to device here */
  this->stop_position_ = position;  // Do not set if write failed
  return;
};

void AS5600Component::write_maximum_angle(uint16_t angle) {
  /* write to device here */
  this->maximum_angle_ = angle;  // Do not set if write failed
  return;
};

/** AS5601 Register Map **/
static const uint8_t REGISTER_ABN = 0x09;
static const uint8_t REGISTER_PUSHTHR = 0x0A;
/** **/

static const char *TAG = "as5601";

void AS5601Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up device...");
  ESP_LOGCONFIG(TAG, "  AB Resolution: %d", this->ab_resolution_);
  ESP_LOGCONFIG(TAG, "  Push Threshold: %d", this->push_threshold_);
  LOG_AS560X(this);
};

void AS5601Component::setup() {
  // if (this->is_device_online()) return this->mark_failed();
  AS560XComponent::common_setup();
  this->write_ab_resolution(this->ab_resolution_);
  this->write_push_threshold(this->push_threshold_);
};

void AS5601Component::write_ab_resolution(uint16_t positions) {
  ESP_LOGI(TAG, "Configuring AB positions from %d to %d", this->ab_resolution_, positions);
  // Taken from https://github.com/bitfasching/AS5601/blob/master/AS5601.h
  char power = -1;
  while ((1 << ++power) < this->ab_resolution_)
    ;
  if (!this->write_byte(REGISTER_ABN, power - 3))
    return;
  this->ab_resolution_ = positions;  // Do not set if write failed
  ESP_LOGI(TAG, "Configured AB positions %d", this->ab_resolution_);
  return;
};

void AS5601Component::write_push_threshold(uint16_t threshold) { return; };

}  // namespace as560x
}  // namespace esphome
