#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint16_t TMC2209::get_sg_result() {
  // TODO: Decide on shifting according to datasheet
  return (uint16_t) tmc2209_readInt(&this->driver_, TMC2209_SG_RESULT);
}

void TMC2209::set_sg_threshold(uint8_t threshold) {
  tmc2209_writeInt(&this->driver_, TMC2209_SGTHRS, (int32_t) threshold);
  this->sgthrs_ = threshold;
}

}  // namespace tmc
}  // namespace esphome
