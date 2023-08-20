#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::blank_time(uint8_t select) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, select);
}

uint8_t TMC2209::microsteps() {
  const auto val = TMC2209_FIELD_READ(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT);
  ESP_LOGI(TAG, "read ms=%d " BYTE_TO_BINARY_PATTERN, val, BYTE_TO_BINARY(val));
  return val;
}

void TMC2209::microsteps(uint8_t ms) {
  ESP_LOGI(TAG, "setting ms=%d " BYTE_TO_BINARY_PATTERN, ms, BYTE_TO_BINARY(ms));
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, ms);
}

}  // namespace tmc
}  // namespace esphome
