#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::stallguard_threshold(uint8_t threshold) {
  tmc2209_writeInt(&this->driver_, TMC2209_SGTHRS, (int32_t) threshold);
  this->sg_thrs_ = threshold;
}

uint16_t TMC2209::stallguard_result() {
  // TODO: Decide on shifting according to datasheet
  return (uint16_t) tmc2209_readInt(&this->driver_, TMC2209_SG_RESULT);
}

float TMC2209::calc_motor_load(uint16_t sg_result) {
  return (510.0 - (float) sg_result) / ((float) this->sg_thrs_ * 2.0);
}

}  // namespace tmc
}  // namespace esphome
