#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace as560x {

#define LOG_AS560X(this) \
  ESP_LOGCONFIG(TAG, "  Zero Position: %d", this->zero_position_); \
  LOG_SENSOR("  ", "Angle Sensor", this->angle_sensor_); \
  LOG_SENSOR("  ", "Magnitude Sensor", this->magnitude_sensor_); \
  LOG_BINARY_SENSOR("  ", "Magnet Presence Binary Sensor", this->presence_binary_sensor_);

class AS560XComponent : public Component, public i2c::I2CDevice {
 public:
  AS560XComponent() = default;

  float get_setup_priority() const override { return setup_priority::DATA; }
  void loop() override;
  void setup() override;

  void set_presence_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    this->presence_binary_sensor_ = binary_sensor;
  }
  void set_angle_sensor(sensor::Sensor *sensor) { this->angle_sensor_ = sensor; }
  void set_magnitude_sensor(sensor::Sensor *sensor) { this->magnitude_sensor_ = sensor; }

  // Configuration setters and writers
  void set_zero_position(uint position) { this->zero_position_ = position; };
  void write_zero_position(uint16_t position);

  // Device readings
  uint16_t magnitude();
  uint16_t angle();
  uint16_t raw_angle();
  uint8_t gain();
  bool presence();

 protected:
  void common_setup();
  int16_t read_status_byte();
  bool is_device_online();

  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *magnitude_sensor_{nullptr};

  uint16_t zero_position_{0};
};

class AS5600Component : public AS560XComponent {
 public:
  AS5600Component() = default;
  void dump_config() override;
  void setup() override;

  // Configuration setters and writers
  void set_stop_position(uint value) { this->stop_position_ = value; };
  void set_maximum_angle(uint value) { this->maximum_angle_ = value; };
  void write_stop_position(uint16_t position);
  void write_maximum_angle(uint16_t angle);

 protected:
  uint16_t stop_position_{4095};
  uint16_t maximum_angle_{359};
};

template<typename... Ts> class AS5600ConfigAction : public Action<Ts...>, public Parented<AS5600Component> {
 public:
  // explicit AS5600ConfigAction(AS5600Component *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint16_t, zero_position)
  TEMPLATABLE_VALUE(uint16_t, stop_position)
  TEMPLATABLE_VALUE(uint16_t, maximum_angle)

  void play(Ts... x) override {
    if (this->zero_position_.has_value())
      this->parent_->write_zero_position(this->zero_position_.value(x...));

    if (this->stop_position.has_value())
      this->parent_->write_stop_position(this->stop_position.value(x...));

    if (this->maximum_angle_.has_value())
      this->parent_->write_maximum_angle(this->maximum_angle_.value(x...));
  }

 protected:
  AS5600Component *parent_;
};

class AS5601Component : public AS560XComponent {
 public:
  AS5601Component() = default;
  void dump_config() override;
  void setup() override;

  // Configuration setters and writers
  void set_ab_resolution(uint16_t value) { this->ab_resolution_ = value; };
  void set_push_threshold(uint value) { this->push_threshold_ = value; };
  void write_ab_resolution(uint16_t resolution);
  void write_push_threshold(uint16_t threshold);

  uint16_t ab_resolution() { return this->ab_resolution_; }

 protected:
  uint16_t ab_resolution_{8};
  uint16_t push_threshold_{0};
};

template<typename... Ts> class AS5601ConfigAction : public Action<Ts...>, public Parented<AS5601Component> {
 public:
  // explicit AS5601ConfigAction(AS5601Component *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint16_t, zero_position)
  TEMPLATABLE_VALUE(uint16_t, ab_resolution)
  TEMPLATABLE_VALUE(uint16_t, push_threshold)

  void play(Ts... x) override {
    if (this->zero_position_.has_value())
      this->parent_->write_zero_position(this->zero_position_.value(x...));

    if (this->ab_resolution_.has_value()) {
      ESP_LOGI("as5601", "Received new AB value %d", this->ab_resolution_.value(x...));
      this->parent_->write_ab_resolution(this->ab_resolution_.value(x...));
    }

    if (this->push_threshold_.has_value())
      this->parent_->write_push_threshold(this->push_threshold_.value(x...));
  }

 protected:
  AS5601Component *parent_;
};

}  // namespace as560x
}  // namespace esphome
