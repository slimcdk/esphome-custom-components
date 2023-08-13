#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

// Enable/disable driver
void TMC2209::enable(bool enable) {
  this->is_enabled_ = enable;
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(enable);
    this->enable_pin_state_ = enable;
  }
}

void TMC2209::enable() { this->enable(true); }
void TMC2209::disable() { this->enable(false); }

uint16_t TMC2209::get_ms_counter() { return (uint16_t) tmc2209_readInt(&this->driver_, TMC2209_MSCNT); };
int16_t TMC2209::get_ms_current_a() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_MSCURACT, TMC2209_CUR_A_MASK, TMC2209_CUR_A_SHIFT);  // TODO
};
int16_t TMC2209::get_ms_current_b() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_MSCURACT, TMC2209_CUR_B_MASK, TMC2209_CUR_B_SHIFT);  // TODO
};

void TMC2209::set_velocity(int32_t velocity) {
  this->enable((bool) velocity);
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, velocity);
}

void TMC2209::set_run_current(int32_t current) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, current);
}
void TMC2209::set_hold_current(int32_t current) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, current);
}
void TMC2209::set_hold_current_delay(int32_t delay) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, delay);
}

}  // namespace tmc
}  // namespace esphome
