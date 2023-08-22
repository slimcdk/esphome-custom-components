#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint16_t TMC2209::gstat() { return tmc2209_readInt(&this->driver_, TMC2209_GSTAT); }
void TMC2209::gstat(uint16_t setting) { return tmc2209_writeInt(&this->driver_, TMC2209_GSTAT, setting); }

bool TMC2209::gstat_reset() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT);
}

void TMC2209::gstat_reset(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT, clear);
}

bool TMC2209::gstat_drv_err() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT);
}

void TMC2209::gstat_drv_err(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT, clear);
}

bool TMC2209::gstat_uv_cp() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT);
}

void TMC2209::gstat_uv_cp(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT, clear);
}

}  // namespace tmc
}  // namespace esphome
