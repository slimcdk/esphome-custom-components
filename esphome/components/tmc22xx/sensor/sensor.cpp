#include "esphome/components/tmc22xx/driver_registers.h"
#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tmc22xx {

// static const char *const TAG = "tmc22xx.sensor";

void StallGuardResultSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX StallGuard Result Sensor", this); }
void StallGuardResultSensor::update() { this->publish_state(this->parent_->read_register(SG_RESULT)); }

void MotorLoadSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX Motor Load Sensor", this); }
void MotorLoadSensor::update() { this->publish_state(this->parent_->get_motor_load() * 100.0f); }

void ActualCurrentSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX Actual Current Sensor", this); }
void ActualCurrentSensor::update() {
  const uint8_t acs = this->parent_->read_field(CS_ACTUAL_FIELD);
  this->publish_state(this->parent_->current_scale_to_rms_current_mA(acs));
}

void PWMScaleSumSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX PWM Scale Sum Sensor", this); }
void PWMScaleSumSensor::update() { this->publish_state(this->parent_->read_field(PWM_SCALE_SUM_FIELD)); }

void PWMScaleAutoSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX PWM Scale Auto Sensor", this); }
void PWMScaleAutoSensor::update() { this->publish_state(this->parent_->read_field(PWM_SCALE_AUTO_FIELD)); }

void PWMOFSAutoSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX OFS Auto Sensor", this); }
void PWMOFSAutoSensor::update() { this->publish_state(this->parent_->read_field(PWM_OFS_AUTO_FIELD)); }

void PWMGradAutoSensor::dump_config() { LOG_SENSOR(" ", "TMC22XX Grad Auto Sensor", this); }
void PWMGradAutoSensor::update() { this->publish_state(this->parent_->read_field(PWM_GRAD_AUTO_FIELD)); }

}  // namespace tmc22xx
}  // namespace esphome
