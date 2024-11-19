#include "esphome/components/tmc2209/tmc2209_api_registers.h"
#include "tmc2209_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tmc2209 {

// static const char *const TAG = "tmc2209.sensor";

void StallGuardResultSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 StallGuard Result Sensor", this); }
void StallGuardResultSensor::update() { this->publish_state(this->parent_->read_register(SG_RESULT)); }

void MotorLoadSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 Motor Load Sensor", this); }
void MotorLoadSensor::update() { this->publish_state(this->parent_->get_motor_load() * 100.0f); }

void ActualCurrentSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 Actual Current Sensor", this); }
void ActualCurrentSensor::update() {
  const uint8_t acs = this->parent_->read_field(CS_ACTUAL_FIELD);
  this->publish_state(this->parent_->current_scale_to_rms_current_mA(acs));
}

void PWMScaleSumSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 PWM Scale Sum Sensor", this); }
void PWMScaleSumSensor::update() { this->publish_state(this->parent_->read_field(PWM_SCALE_SUM_FIELD)); }

void PWMScaleAutoSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 PWM Scale Auto Sensor", this); }
void PWMScaleAutoSensor::update() { this->publish_state(this->parent_->read_field(PWM_SCALE_AUTO_FIELD)); }

void PWMOFSAutoSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 OFS Auto Sensor", this); }
void PWMOFSAutoSensor::update() { this->publish_state(this->parent_->read_field(PWM_OFS_AUTO_FIELD)); }

void PWMGradAutoSensor::dump_config() { LOG_SENSOR(" ", "TMC2209 Grad Auto Sensor", this); }
void PWMGradAutoSensor::update() { this->publish_state(this->parent_->read_field(PWM_GRAD_AUTO_FIELD)); }

}  // namespace tmc2209
}  // namespace esphome
