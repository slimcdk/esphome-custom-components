#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint16_t TMC2209::read_gstat() { return tmc2209_readInt(&this->driver_, TMC2209_GSTAT); }
void TMC2209::write_gstat(uint16_t setting) { return tmc2209_writeInt(&this->driver_, TMC2209_GSTAT, setting); }

void TMC2209::read_gstat_update() {
  this->gstat_ = this->read_gstat();
  this->gstat_last_read_ = (time_t) millis();
}

void TMC2209::write_gstat_update() {
  this->write_gstat(this->gstat_);
  this->gstat_last_write_ = (time_t) millis();
}

bool TMC2209::read_gstat_reset() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT);
}

void TMC2209::write_gstat_reset(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT, clear);
}

bool TMC2209::read_gstat_drv_err() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT);
}

void TMC2209::write_gstat_drv_err(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT, clear);
}

bool TMC2209::read_gstat_uv_cp() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT);
}

void TMC2209::write_gstat_uv_cp(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT, clear);
}

bool TMC2209::get_gstat_reset() { return this->read_gstat_reset(); }               // TODO: use shadow register
void TMC2209::set_gstat_reset(bool clear) { this->write_gstat_reset(clear); }      // TODO: use shadow register
bool TMC2209::get_gstat_drv_err() { return this->read_gstat_drv_err(); }           // TODO: use shadow register
void TMC2209::set_gstat_drv_err(bool clear) { this->write_gstat_drv_err(clear); }  // TODO: use shadow register
bool TMC2209::get_gstat_uv_cp() { return this->read_gstat_uv_cp(); }               // TODO: use shadow register
void TMC2209::set_gstat_uv_cp(bool clear) { this->write_gstat_uv_cp(clear); }      // TODO: use shadow register

}  // namespace tmc
}  // namespace esphome
