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
  void update() override { this->publish_state(this->parent_->read_temp()); }
};

class SupplyVoltageSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 Supply Voltage Sensor", this); }
  void update() override { this->publish_state(this->parent_->read_supply_voltage()); }
};

class ADCVoltageSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 ADC Voltage Sensor", this); }
  void update() override { this->publish_state(this->parent_->read_adc_ain()); }
};

class EncoderPosSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 Encoder Position Sensor", this); }
  void update() override { this->publish_state((int32_t) this->parent_->read_field(TMC5240_X_ENC_FIELD)); }
};

class StallGuardResultSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 StallGuard Result Sensor", this); }
  void update() override { this->publish_state(this->parent_->read_field(TMC5240_SG4_RESULT_FIELD)); }
};

class MotorLoadSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC5240Stepper> {
  void dump_config() { LOG_SENSOR(" ", "TMC5240 Motor Load Sensor", this); }
  void update() override { this->publish_state(this->parent_->get_motor_load() * 100.0); }
};

}  // namespace tmc5240
}  // namespace esphome
