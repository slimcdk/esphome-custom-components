#include "tmc2209.h"

#include "esphome/core/log.h"

namespace esphome {
namespace tmc {

static const char *const TAG = "tmc2209.sensor";

float TMC2209Sensor::get_setup_priority() const { return setup_priority::DATA; }

void TMC2209Sensor::dump_config() {
  LOG_SENSOR("", "TMC2209 Sensor", this);
  LOG_UPDATE_INTERVAL(this);
}

void TMC2209Sensor::update() { this->publish_state(this->tmc2209->address()); }

}  // namespace tmc
}  // namespace esphome
