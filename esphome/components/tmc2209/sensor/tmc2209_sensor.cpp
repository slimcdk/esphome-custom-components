#include "tmc2209_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace tmc {

static const char *const TAG = "tmc2209.sensor";

void TMC2209Sensor::dump_config() {
  LOG_SENSOR("", "TMC2209 Sensor", this);
  LOG_SENSOR("  ", "Stallguard result", this->sg_result_sensor_);

  LOG_UPDATE_INTERVAL(this);
}

void TMC2209Sensor::setup() {}

void TMC2209Sensor::update() { sg_result_sensor_->publish_state(0); }

}  // namespace tmc
}  // namespace esphome
