#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint32_t TMC2209::read_ioin() { return tmc2209_readInt(&this->driver_, TMC2209_IOIN); }

void TMC2209::update_ioin() {
  this->ioin_ = this->read_ioin();
  this->ioin_last_read_ = (time_t) millis();
}

bool TMC2209::get_ioin_enn() { return (bool) (((this->ioin_) & (TMC2209_ENN_MASK)) >> (TMC2209_ENN_SHIFT)); }

bool TMC2209::get_ioin_ms1() { return (bool) (((this->ioin_) & (TMC2209_IOIN)) >> (TMC2209_MS2_SHIFT)); }

bool TMC2209::get_ioin_ms2() { return (bool) (((this->ioin_) & (TMC2209_MS2_MASK)) >> (TMC2209_MS2_SHIFT)); }

bool TMC2209::get_ioin_diag() { return (bool) (((this->ioin_) & (TMC2209_DIAG_MASK)) >> (TMC2209_DIAG_SHIFT)); }

bool TMC2209::get_ioin_pdn_uart() {
  return (bool) (((this->ioin_) & (TMC2209_PDN_UART_MASK)) >> (TMC2209_PDN_UART_SHIFT));
}

bool TMC2209::get_ioin_step() { return (bool) (((this->ioin_) & (TMC2209_STEP_MASK)) >> (TMC2209_STEP_SHIFT)); }

bool TMC2209::get_ioin_spread_en() { return (bool) (((this->ioin_) & (TMC2209_SEL_A_MASK)) >> (TMC2209_SEL_A_SHIFT)); }

bool TMC2209::get_ioin_dir() { return (bool) (((this->ioin_) & (TMC2209_DIR_MASK)) >> (TMC2209_DIR_SHIFT)); }

int8_t TMC2209::get_chip_version() {
  return (uint8_t) (((this->ioin_) & (TMC2209_VERSION_MASK)) >> (TMC2209_VERSION_SHIFT));
}
int8_t TMC2209::read_chip_version() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
