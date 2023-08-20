#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void TMC2209::fclktrim(uint8_t fclktrim) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_FCLKTRIM_MASK, TMC2209_FCLKTRIM_SHIFT, fclktrim);
}

uint8_t TMC2209::fclktrim() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_FCLKTRIM_MASK, TMC2209_FCLKTRIM_SHIFT);
}

void TMC2209::ottrim(uint8_t ottrim) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_OTTRIM_MASK, TMC2209_OTTRIM_SHIFT, ottrim);
}

uint8_t TMC2209::ottrim() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_OTTRIM_MASK, TMC2209_OTTRIM_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
