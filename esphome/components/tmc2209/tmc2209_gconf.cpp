#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::pdn_disable(bool disable) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, disable);
}

void TMC2209::use_mres_register(bool use) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_GCONF, TMC2209_MSTEP_REG_SELECT_MASK, TMC2209_MSTEP_REG_SELECT_SHIFT,
                      use);
}

bool TMC2209::has_inverse_direction() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT);
}

void TMC2209::set_inverse_direction(bool inverse_direction) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, inverse_direction);
}

}  // namespace tmc
}  // namespace esphome
