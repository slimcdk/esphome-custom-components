#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::tcool_threshold(int32_t threshold) { tmc2209_writeInt(&this->driver_, TMC2209_TCOOLTHRS, threshold); }

}  // namespace tmc
}  // namespace esphome
