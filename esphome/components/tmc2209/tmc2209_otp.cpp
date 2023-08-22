#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint32_t TMC2209::otpread() { return tmc2209_readInt(&this->driver_, TMC2209_OTP_READ); }

bool TMC2209::optread_en_spreadcycle() {
  return (bool) TMC2209_FIELD_READ(&this->driver_, TMC2209_OTP_READ, TMC2209_STST_MASK, TMC2209_STST_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
