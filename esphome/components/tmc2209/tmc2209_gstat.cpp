#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

bool TMC2209::has_reset_since_last_gstat_read() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT);
}

bool TMC2209::has_driver_error() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT);
}

int32_t TMC2209::get_driver_status() { return tmc2209_readInt(&this->driver_, TMC2209_DRVSTATUS); }

bool TMC2209::undervoltage_detection() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
