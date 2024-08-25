#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240 {

static const char *TAG = "tmc5240.stepper";

TMC5240Stepper::TMC5240Stepper() {
  this->id_ = component_index++;
  components[this->id_] = this;
}

void TMC5240Stepper::setup() {
  this->enn_pin_->setup();
  this->diag0_pin_->setup();
  this->diag1_pin_->setup();

  this->enn_pin_->digital_write(true);

  this->set_vmax(this->max_speed_);
  this->set_amax(this->acceleration_);
  this->set_dmax(this->deceleration_);

  this->set_enc_const(25);
  this->enable_encoder_position(true);
}

void TMC5240Stepper::loop() {
  this->current_position = this->get_xactual();

  if (!this->has_reached_target()) {
    this->enn_pin_->digital_write(false);
    this->set_vmax(this->max_speed_);
    this->set_amax(this->acceleration_);
    this->set_dmax(this->deceleration_);
    // TODO: tmc5240_moveTo(&this->driver_, this->target_position, this->max_speed_);
  } else {
    this->enn_pin_->digital_write(true);
  }
}

uint16_t TMC5240Stepper::get_microsteps() {
  const uint8_t mres = this->chopconf_mres();
  return 256 >> mres;
}

void TMC5240Stepper::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->chopconf_mres(mres);
}

/** setters */
void TMC5240Stepper::set_vmax(int32_t max) { this->write_field(TMC5240_VMAX_FIELD, max); }
void TMC5240Stepper::set_amax(int32_t max) { this->write_field(TMC5240_AMAX_FIELD, max); }
void TMC5240Stepper::set_dmax(int32_t max) { this->write_field(TMC5240_DMAX_FIELD, max); }
void TMC5240Stepper::set_tvmax(int32_t max) { this->write_field(TMC5240_TVMAX_FIELD, max); }
void TMC5240Stepper::enable_encoder_position(bool enable) { this->write_field(TMC5240_LATCH_X_ACT_FIELD, enable); }
void TMC5240Stepper::set_xactual(int32_t value) { this->write_field(TMC5240_XACTUAL_FIELD, value); }
void TMC5240Stepper::set_shaft_direction(bool invert) { this->write_field(TMC5240_SHAFT_FIELD, invert); }
void TMC5240Stepper::chopconf_mres(uint8_t index) { this->write_field(TMC5240_MRES_FIELD, index); }
void TMC5240Stepper::enc_deviation(uint32_t deviation) { this->write_field(TMC5240_ENC_DEVIATION_FIELD, deviation); }

void TMC5240Stepper::set_enc_const(float value) {
  int16_t factor = (int16_t) value;
  int16_t fraction = (value - factor) * 10;
  int32_t value_ = _16_32(factor, fraction);
  this->write_field(TMC5240_ENC_CONST_FIELD, value_);
}

/** getters */
bool TMC5240Stepper::get_using_external_oscillator() { return this->read_field(TMC5240_EXT_CLK_FIELD); }
bool TMC5240Stepper::get_adc_err() { return this->read_field(TMC5240_ADC_ERR_FIELD); }
bool TMC5240Stepper::get_shaft_direction() { return this->read_field(TMC5240_SHAFT_FIELD); }
bool TMC5240Stepper::enable_encoder_position() { return this->read_field(TMC5240_LATCH_X_ACT_FIELD); }
uint8_t TMC5240Stepper::get_encmode_ignore_ab() { return this->read_field(TMC5240_IGNORE_AB_FIELD); }
uint8_t TMC5240Stepper::get_version() { return this->read_field(TMC5240_VERSION_FIELD); }
uint8_t TMC5240Stepper::get_revision() { return this->read_field(TMC5240_SILICON_RV_FIELD); }
uint8_t TMC5240Stepper::chopconf_mres() { return this->read_field(TMC5240_MRES_FIELD); }
int32_t TMC5240Stepper::get_xactual() { return this->read_field(TMC5240_XACTUAL_FIELD); }
int32_t TMC5240Stepper::get_sg4_result() { return this->read_field(TMC5240_SG4_RESULT_FIELD); }
int32_t TMC5240Stepper::get_x_enc() { return this->read_field(TMC5240_X_ENC_FIELD); }

float TMC5240Stepper::get_vsupply() {
  int32_t adc_value = this->read_field(TMC5240_ADC_VSUPPLY_FIELD);
  return (float) adc_value * 9.732;
}

float TMC5240Stepper::get_temp() {
  int32_t adc_value = this->read_field(TMC5240_ADC_TEMP_FIELD);
  return (adc_value - 2039) / 7.7;
}

int32_t TMC5240Stepper::get_vactual() {
  int32_t value = this->read_field(TMC5240_VACTUAL_FIELD);
  return CAST_Sn_TO_S32(value, 23);
}

float TMC5240Stepper::get_enc_const() {
  int32_t value = this->read_field(TMC5240_ENC_CONST_FIELD);
  int16_t factor = SHORT(value, 1);
  int16_t fraction = SHORT(value, 0);
  return (float) factor + (float) fraction / 10.0f;
}

uint32_t TMC5240Stepper::enc_deviation() {
  uint32_t value = this->read_field(TMC5240_ENC_DEVIATION_FIELD);
  return value;  // CAST_Sn_TO_S32(value, 19);
}

extern "C" {

TMC5240BusType tmc5240_getBusType(uint16_t id) {
  TMC5240Stepper *comp = components[id];
  return comp->get_bus_type();
}

void tmc5240_readWriteSPI(uint16_t id, uint8_t *data, size_t dataLength) {
#if defined(TMC5240_USE_SPI)
  auto *comp = static_cast<TMC5240SPIStepper *>(tmc5240::components[id]);

  if (comp == nullptr) {
    ESP_LOGE(TAG, "Component with id %d is null", id);
    return;
  }

  comp->enable();
  comp->transfer_array(data, dataLength);
  comp->disable();
#endif
}

uint8_t tmc5240_getNodeAddress(uint16_t id) {
#if defined(TMC5240_USE_UART)
  auto *comp = static_cast<TMC5240UARTStepper *>(tmc5240::components[id]);
  return comp->get_uart_address();
#else
  return 0;
#endif
}

bool tmc5240_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
#if defined(TMC5240_USE_UART)
  auto *comp = static_cast<TMC5240UARTStepper *>(tmc5240::components[id]);

  if (comp == nullptr) {
    ESP_LOGE(TAG, "Component with id %d is null", id);
    return false;
  }

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    comp->read_array(data, readLength);
  }
  return true;
#else
  return false;
#endif
}
}

}  // namespace tmc5240
}  // namespace esphome
