#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint32_t TMC2209::ioin() { return tmc2209_readInt(&this->driver_, TMC2209_IOIN); }

bool TMC2209::ioin_enn() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_ENN_MASK, TMC2209_ENN_SHIFT);
}

bool TMC2209::ioin_ms1() { return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_IOIN, TMC2209_MS2_SHIFT); }

bool TMC2209::ioin_ms2() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_MS2_MASK, TMC2209_MS2_SHIFT);
}

bool TMC2209::ioin_diag() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIAG_MASK, TMC2209_DIAG_SHIFT);
}

bool TMC2209::ioin_pdn_uart() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_PDN_UART_MASK, TMC2209_PDN_UART_SHIFT);
}

bool TMC2209::ioin_step() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_STEP_MASK, TMC2209_STEP_SHIFT);
}

bool TMC2209::ioin_spread_en() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_SEL_A_MASK, TMC2209_SEL_A_SHIFT);
}

bool TMC2209::ioin_dir() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIR_MASK, TMC2209_DIR_SHIFT);
}

int8_t TMC2209::ioin_chip_version() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
