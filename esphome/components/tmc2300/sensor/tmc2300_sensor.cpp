#include "tmc2300_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace tmc2300 {

// static const char *const TAG = "tmc2300.sensor";

void StallGuardResultSensor::dump_config() { LOG_SENSOR(" ", "TMC2300 StallGuard Result Sensor", this); }
void StallGuardResultSensor::update() { this->publish_state(this->parent_->read_register(TMC2300_SG_VALUE)); }

void MotorLoadSensor::dump_config() { LOG_SENSOR(" ", "TMC2300 Motor Load Sensor", this); }
void MotorLoadSensor::update() { this->publish_state(this->parent_->get_motor_load() * 100.0f); }

void ActualCurrentSensor::dump_config() { LOG_SENSOR(" ", "TMC2300 Actual Current Sensor", this); }
void ActualCurrentSensor::update() {
  const uint8_t acs = this->parent_->read_field(TMC2300_CS_ACTUAL_FIELD);
  this->publish_state(this->parent_->current_scale_to_rms_current_mA(acs));
}

}  // namespace tmc2300
}  // namespace esphome
