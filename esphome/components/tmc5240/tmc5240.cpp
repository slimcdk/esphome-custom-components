#include "tmc5240.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc5240 {

void TMC5240::setup() {
  if (this->enn_pin_) {
    this->enn_pin_->setup();
    this->enn_pin_->digital_write(false);
  }
  if (this->diag0_pin_) {
    this->diag0_pin_->setup();
  }
  if (this->diag1_pin_) {
    this->diag1_pin_->setup();
  }

  const bool adc_err = this->read_field(TMC5240_ADC_ERR_FIELD);
  if (adc_err) {
    this->status_set_warning("tmc5240 adc error");
  }
}

void TMC5240::loop() { ESP_LOGD(TAG, "%.f", this->read_supply_voltage_mV()); }

void TMC5240::dump_config() {
  LOG_PIN("  ENN Pin: ", this->enn_pin_);
  LOG_PIN("  DIAG0 Pin: ", this->diag0_pin_);
  LOG_PIN("  DIAG1 Pin: ", this->diag1_pin_);

  const uint8_t icv_ = this->read_field(TMC5240_VERSION_FIELD);
  const uint8_t icr_ = this->read_field(TMC5240_SILICON_RV_FIELD);
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X (revision: 0x%02X)", icv_, icr_);

  const bool adc_err = this->read_field(TMC5240_ADC_ERR_FIELD);
  if (adc_err) {
    ESP_LOGE(TAG, "  Faulty ADC detected! Advised not to use ADC type input!");
    this->status_set_warning("tmc5240 adc error");
  }
  ESP_LOGCONFIG(TAG, "  Supply Voltage: %.f mV", this->read_supply_voltage_mV());

  this->write_register(TMC5240_VACTUAL, 1000);
}

uint32_t TMC5240::read_field(RegisterField field) {
  uint32_t value = read_register(field.address);
  return this->extract_field(value, field);
}

void TMC5240::write_field(RegisterField field, uint32_t value) {
  // read register, update field and write register
  int32_t register_value = this->read_register(field.address);
  int32_t updated_value = (register_value & (~field.mask)) | ((register_value << field.shift) & field.mask);
  this->write_register(field.address, updated_value);
};

uint32_t TMC5240::extract_field(uint32_t data, RegisterField field) {
  uint32_t value = (data & field.mask) >> field.shift;

  if (field.isSigned) {
    // Apply sign conversion
    uint32_t baseMask = field.mask >> field.shift;
    uint32_t signMask = baseMask & (~baseMask >> 1);
    value = (value ^ signMask) - signMask;
  }

  return value;
}

float TMC5240::read_supply_voltage_mV() { return (float) this->read_field(TMC5240_ADC_VSUPPLY_FIELD) * 9.732; }
float TMC5240::read_adc_ain_mV() { return (float) this->read_field(TMC5240_ADC_AIN_FIELD) * 305.2; }
float TMC5240::read_temp_C() { return ((float) this->read_field(TMC5240_ADC_TEMP_FIELD) - 2039) / 7.7; }
int32_t TMC5240::get_vactual() { return CAST_Sn_TO_S32(this->read_register(TMC5240_VACTUAL), 23); }

}  // namespace tmc5240
}  // namespace esphome
