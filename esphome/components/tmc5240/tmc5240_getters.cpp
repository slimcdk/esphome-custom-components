#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240 {

static const char *TAG = "tmc5240";

float TMC5240::get_vsupply() {
  int32_t adc_value =
      TMC5240_FIELD_READ(&this->driver_, TMC5240_ADC_VSUPPLY_AIN, TMC5240_ADC_VSUPPLY_MASK, TMC5240_ADC_VSUPPLY_SHIFT);
  return (float) adc_value * 9.732;
}

float TMC5240::get_temp() {
  int32_t adc_value =
      TMC5240_FIELD_READ(&this->driver_, TMC5240_ADC_TEMP, TMC5240_ADC_TEMP_MASK, TMC5240_ADC_TEMP_SHIFT);
  return (adc_value - 2039) / 7.7;
}

int32_t TMC5240::get_xactual() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_XACTUAL, TMC5240_XACTUAL_MASK, TMC5240_XACTUAL_SHIFT);
}

int32_t TMC5240::get_vactual() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_VACTUAL, TMC5240_VACTUAL_MASK, TMC5240_VACTUAL_SHIFT);
}

}  // namespace tmc5240
}  // namespace esphome
