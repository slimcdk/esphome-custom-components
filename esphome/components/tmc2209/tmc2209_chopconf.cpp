#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::blank_time(uint8_t select) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, select);
}

}  // namespace tmc
}  // namespace esphome
