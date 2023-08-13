#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::set_tcool_threshold(int32_t threshold) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_TCOOLTHRS, TMC2209_TCOOLTHRS, TMC2209_TCOOLTHRS, threshold);
}

}  // namespace tmc
}  // namespace esphome
