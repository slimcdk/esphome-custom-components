#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint8_t TMC2209::transmission_counter() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IFCNT, TMC2209_IFCNT_MASK, TMC2209_IFCNT_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
