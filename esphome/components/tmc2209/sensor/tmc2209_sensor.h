#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/tmc2209/tmc2209.h"

namespace esphome {
namespace tmc2209 {

static const char *const TAG = "tmc2209.sensor";

class StallGuardResultSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2209> {
  void dump_config() { LOG_SENSOR(" ", "TMC2209 StallGuard Result Sensor", this); }
  void update() override { this->publish_state((uint16_t) this->parent_->read_register(TMC2209_SG_RESULT)); }
};

class MotorLoadSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2209> {
  void dump_config() { LOG_SENSOR(" ", "TMC2209 Motor Load Sensor", this); }
  void update() override { this->publish_state(this->parent_->get_motor_load() * 100.0); }
};

}  // namespace tmc2209
}  // namespace esphome
