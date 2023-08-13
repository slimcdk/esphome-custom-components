#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

bool TMC2209::get_ioin_enn_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_ENN_MASK, TMC2209_ENN_SHIFT);
}

bool TMC2209::get_ioin_ms1_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_MS1_MASK, TMC2209_MS2_SHIFT);
}

bool TMC2209::get_ioin_ms2_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_MS2_MASK, TMC2209_MS2_SHIFT);
}

bool TMC2209::get_ioin_diag_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIAG_MASK, TMC2209_DIAG_SHIFT);
}

bool TMC2209::get_ioin_pdn_uart_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_PDN_UART_MASK, TMC2209_PDN_UART_SHIFT);
}

bool TMC2209::get_ioin_step_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_STEP_MASK, TMC2209_STEP_SHIFT);
}

bool TMC2209::get_ioin_spread_en_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_SEL_A_MASK, TMC2209_SEL_A_SHIFT);
}

bool TMC2209::get_ioin_dir_state() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIR_MASK, TMC2209_DIR_SHIFT);
}

int8_t TMC2209::get_version() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
