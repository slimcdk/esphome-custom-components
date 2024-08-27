#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/tmc5240/tmc5240.h"

namespace esphome {
namespace tmc5240 {

static const char *const TAG = "tmc5240.sensor";

class TemperatureSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 Temperature Sensor", this); }
  void update() override { this->publish_state(this->parent_->get_temp()); }
};

class VoltageSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 Voltage Sensor", this); }
  void update() override { this->publish_state(this->parent_->get_vsupply()); }
};

class EncoderPosSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 Encoder Position Sensor", this); }
  void update() override { this->publish_state(this->parent_->get_x_enc()); }
};

class StallGuardResultSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 StallGuard Result Sensor", this); }
  void update() override { this->publish_state(this->parent_->get_sg4_result()); }
};

}  // namespace tmc5240
}  // namespace esphome
