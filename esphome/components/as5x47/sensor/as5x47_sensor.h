#pragma once

#include "esphome/components/as5x47/as5x47.h"

namespace esphome {
namespace as5x47 {

class AngleSensor : public PollingComponent, public sensor::Sensor, public Parented<AS5X47Component> {
  void dump_config() { LOG_SENSOR(" ", "AS5X47 Angle Sensor", this); }
  void update() override { this->publish_state(/*this->parent_->read_temp()*/); }
};

}  // namespace as5x47
}  // namespace esphome
