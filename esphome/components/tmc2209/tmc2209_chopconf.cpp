#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::set_blank_time(uint8_t select) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, select);
}

uint8_t TMC2209::get_microsteps() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT);
}

void TMC2209::set_microsteps(uint8_t ms) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, ms);
}

}  // namespace tmc
}  // namespace esphome
