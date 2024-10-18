#pragma once

#include "esphome/components/as5047/as5047.h"

namespace esphome {
namespace as5047 {

class AngleSensor : public PollingComponent, public sensor::Sensor, public Parented<AS5047Component> {
  void dump_config() { LOG_SENSOR(" ", "AS5047 Angle Sensor", this); }
  void update() override { this->publish_state(/*this->parent_->read_angle()*/); }
};

}  // namespace as5047
}  // namespace esphome
