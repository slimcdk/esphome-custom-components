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

void TMC5240::set_enc_const(float value) {
  int16_t factor = (int16_t) value;
  int16_t fraction = (value - factor) * 10;
  int32_t value_ = _16_32(factor, fraction);
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_ENC_CONST, TMC5240_ENC_CONST_MASK, TMC5240_ENC_CONST_SHIFT, value_);
}

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
  int32_t value = TMC5240_FIELD_READ(&this->driver_, TMC5240_VACTUAL, TMC5240_VACTUAL_MASK, TMC5240_VACTUAL_SHIFT);
  return CAST_Sn_TO_S32(value, 23);
}

int32_t TMC5240::get_sg4_result() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_SG4_RESULT, TMC5240_SG4_RESULT_MASK, TMC5240_SG4_RESULT_SHIFT);
}

float TMC5240::get_enc_const() {
  int32_t value =

      TMC5240_FIELD_READ(&this->driver_, TMC5240_ENC_CONST, TMC5240_ENC_CONST_MASK, TMC5240_ENC_CONST_SHIFT);

  int16_t factor = SHORT(value, 1);
  int16_t fraction = SHORT(value, 0);

  return (float) factor + (float) fraction / 10.0f;
}

int32_t TMC5240::get_x_enc() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_XENC, TMC5240_X_ENC_MASK, TMC5240_X_ENC_SHIFT);
}

uint8_t TMC5240::get_encmode_ignore_ab() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_ENCMODE, TMC5240_IGNORE_AB_MASK, TMC5240_IGNORE_AB_SHIFT);
}

uint8_t TMC5240::get_version() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_INP_OUT, TMC5240_VERSION_MASK, TMC5240_VERSION_SHIFT);
}

uint8_t TMC5240::get_revision() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_INP_OUT, TMC5240_SILICON_RV_MASK, TMC5240_SILICON_RV_SHIFT);
}

bool TMC5240::get_using_external_oscillator() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_INP_OUT, TMC5240_EXT_CLK_MASK, TMC5240_EXT_CLK_SHIFT);
}

bool TMC5240::get_adc_err() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_INP_OUT, TMC5240_ADC_ERR_MASK, TMC5240_ADC_ERR_SHIFT);
}

void TMC5240::set_shaft_direction(bool invert) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_INP_OUT, TMC5240_SHAFT_MASK, TMC5240_SHAFT_SHIFT, invert);
}

bool TMC5240::get_shaft_direction() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_INP_OUT, TMC5240_SHAFT_MASK, TMC5240_SHAFT_SHIFT);
}

void TMC5240::enable_encoder_position(bool enable) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_ENCMODE, TMC5240_LATCH_X_ACT_MASK, TMC5240_LATCH_X_ACT_SHIFT, enable);
}

bool TMC5240::enable_encoder_position() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_ENCMODE, TMC5240_LATCH_X_ACT_MASK, TMC5240_LATCH_X_ACT_SHIFT);
}

void TMC5240::chopconf_mres(uint8_t index) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_CHOPCONF, TMC5240_MRES_MASK, TMC5240_MRES_SHIFT, index);
}

uint8_t TMC5240::chopconf_mres() {
  return TMC5240_FIELD_READ(&this->driver_, TMC5240_CHOPCONF, TMC5240_MRES_MASK, TMC5240_MRES_SHIFT);
}

uint32_t TMC5240::enc_deviation() {
  uint32_t value = TMC5240_FIELD_READ(&this->driver_, TMC5240_ENC_DEVIATION, TMC5240_ENC_DEVIATION_MASK,
                                      TMC5240_ENC_DEVIATION_SHIFT);
  return value;  // CAST_Sn_TO_S32(value, 19);
}

void TMC5240::enc_deviation(uint32_t deviation) {
  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_ENC_DEVIATION, TMC5240_ENC_DEVIATION_MASK, TMC5240_ENC_DEVIATION_SHIFT,
                      deviation);
}

}  // namespace tmc5240
}  // namespace esphome
