#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint32_t TMC2209::read_otpread() { return tmc2209_readInt(&this->driver_, TMC2209_OTP_READ); }

void TMC2209::update_otpread() {
  this->otpread_ = this->read_otpread();
  this->otpread_last_read_ = (time_t) millis();
}

bool TMC2209::get_optread_en_spreadcycle() {
  return (bool) (((this->drv_status_) & (TMC2209_STST_MASK)) >> (TMC2209_STST_SHIFT));
}

}  // namespace tmc
}  // namespace esphome
