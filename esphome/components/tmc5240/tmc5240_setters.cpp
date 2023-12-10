#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240 {

static const char *TAG = "tmc5240";

void TMC5240::set_vmax(int32_t max) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_VMAX, TMC5240_VMAX_MASK, TMC5240_VMAX_SHIFT, max);
}

void TMC5240::set_amax(int32_t max) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_AMAX, TMC5240_AMAX_MASK, TMC5240_AMAX_SHIFT, max);
}

void TMC5240::set_dmax(int32_t max) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_DMAX, TMC5240_DMAX_MASK, TMC5240_DMAX_SHIFT, max);
}

void TMC5240::set_tvmax(int32_t max) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_TVMAX, TMC5240_TVMAX_MASK, TMC5240_TVMAX_SHIFT, max);
}

void TMC5240::set_xactual(int32_t value) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_XACTUAL, TMC5240_XACTUAL_MASK, TMC5240_XACTUAL_SHIFT, value);
}

// void TMC5240::set_enc_const(float value) {
//   TMC5240_FIELD_WRITE(&this->driver_, TMC5240_ENC_CONST, TMC5240_ENC_CONST_MASK, TMC5240_ENC_CONST_SHIFT, value);
// }

}  // namespace tmc5240
}  // namespace esphome
