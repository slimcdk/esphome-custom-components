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

void TMC5240Stepper::dump_config() {
  LOG_TMC5240(this);
  LOG_STEPPER(this);
}

void TMC5240Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC5240...");

  this->enn_pin_->setup();
  this->diag0_pin_->setup();
  this->diag1_pin_->setup();

  this->enable_driver(false);

  for (uint8_t i = 0; i < TMC5240_REGISTER_COUNT; i++) {
    this->write_register(i, tmc5240_sampleRegisterPreset[i]);
  }

  this->write_field(TMC5240_VMAX_FIELD, (int32_t) this->max_speed_);
  this->write_field(TMC5240_AMAX_FIELD, (int32_t) this->acceleration_);
  this->write_field(TMC5240_DMAX_FIELD, (int32_t) this->deceleration_);

  //
  // this->set_interval(1, [this]() { this->current_position = this->read_field(TMC5240_XACTUAL_FIELD); });

  // this->set_enc_const(25);
  // this->enable_encoder_position(true);

  this->diag0_pin_->setup();
  this->diag0_pin_->attach_interrupt(ISRStore::pin_isr, &this->diag0_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->diag0_isr_store_.pin_triggered_ptr = &this->diag0_triggered_;

  this->diag0_handler_.set_callback([this]() { this->on_alert_callback_.call(DIAG0_TRIGGERED); });

  ESP_LOGCONFIG(TAG, "TMC5240 setup done.");
}

void TMC5240Stepper::loop() {
  this->current_position = this->read_field(TMC5240_XACTUAL_FIELD);

  this->diag0_handler_.check(this->diag0_triggered_);
  this->diag0_triggered_ = false;

  if (!this->has_reached_target()) {
    this->enable_driver(true);

    this->write_register(TMC5240_RAMPMODE, TMC5240_MODE_POSITION);
    this->write_field(TMC5240_VMAX_FIELD, (int32_t) this->max_speed_);
    this->write_field(TMC5240_AMAX_FIELD, (int32_t) this->acceleration_);
    this->write_field(TMC5240_DMAX_FIELD, (int32_t) this->deceleration_);
    this->write_register(TMC5240_XTARGET, this->target_position);
  } else {
    this->enable_driver(false, 1000);
  }
}

#define DRIVER_ACTIVATION "driver_activation"

void TMC5240Stepper::enable_driver(bool enable, uint32_t delay) {
  this->enn_pin_->digital_write(!enable);
  /*
  // Check if the new state or delay is different from the current one
  bool new_state = (this->driver_state_ != enable);
  bool new_delay = (this->current_delay_ != delay);

  // If neither state nor delay has changed, do nothing
  if (!new_state && !new_delay) {
    return;
  }

  // Cancel any existing scheduled state change
  if (this->driver_state_change_scheduled_) {
    this->cancel_timeout(DRIVER_ACTIVATION);
    this->driver_state_change_scheduled_ = false;
  }

  // Update the current delay
  this->current_delay_ = delay;

  // If delay is 0, apply the state change immediately
  if (delay == 0) {
    this->enn_pin_->digital_write(!enable);
    this->driver_state_ = enable;
    ESP_LOGV(TAG, "ENN state changed to %d immediately at %ld", enable, millis());
    return;
  }

  // Schedule a state change after the specified delay
  this->driver_state_change_scheduled_ = true;
  this->set_timeout(DRIVER_ACTIVATION, delay, [this, enable]() {
    this->enn_pin_->digital_write(!enable);
    this->driver_state_ = enable;
    this->driver_state_change_scheduled_ = false;
    ESP_LOGV(TAG, "Scheduled ENN change to %d executed at %ld", enable, millis());
  });
  ESP_LOGV(TAG, "Scheduled ENN change to %d in %ld ms from %ld", enable, delay, millis());
  */
}

uint16_t TMC5240Stepper::get_microsteps() {
  const uint8_t mres = (uint8_t) this->read_field(TMC5240_MRES_FIELD);
  return 256 >> mres;
}

void TMC5240Stepper::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->write_field(TMC5240_MRES_FIELD, mres);
}

void TMC5240Stepper::set_enc_const(float value) {
  int16_t factor = (int16_t) value;
  int16_t fraction = (value - factor) * 10;
  int32_t value_ = _16_32(factor, fraction);
  this->write_field(TMC5240_ENC_CONST_FIELD, value_);
}

float TMC5240Stepper::read_supply_voltage() {
  int32_t adc_value = this->read_field(TMC5240_ADC_VSUPPLY_FIELD);
  return (float) adc_value * 9.732;
}

float TMC5240Stepper::read_adc_ain() {
  int32_t adc_value = this->read_field(TMC5240_ADC_AIN_FIELD);
  return (float) adc_value * 305.2;
}

float TMC5240Stepper::read_temp() {
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

uint32_t TMC5240Stepper::get_enc_deviation() {
  uint32_t value = this->read_field(TMC5240_ENC_DEVIATION_FIELD);
  return value;  // CAST_Sn_TO_S32(value, 19);
}

float TMC5240Stepper::get_motor_load() {
  const uint16_t result = this->read_field(TMC5240_SG4_RESULT_FIELD);
  return (float) (510 - result) / (float) (510 - this->read_field(TMC5240_SGT_FIELD) * 2);
}

/** TMC-API wrappers */
extern "C" {
TMC5240BusType tmc5240_getBusType(uint16_t id) {
  TMC5240Stepper *comp = components[id];
  return comp->get_bus_type();
}

void tmc5240_readWriteSPI(uint16_t id, uint8_t *data, size_t dataLength) {
#if defined(TMC5240_USE_SPI)
  auto *comp = static_cast<TMC5240SPIStepper *>(components[id]);
  comp->enable();
  comp->transfer_array(data, dataLength);
  comp->disable();
#endif
}

uint8_t tmc5240_getNodeAddress(uint16_t id) {
#if defined(TMC5240_USE_UART)
  auto *comp = static_cast<TMC5240UARTStepper *>(components[id]);
  return comp->get_uart_address();
#else
  return 0;
#endif
}

bool tmc5240_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
#if defined(TMC5240_USE_UART)
  auto *comp = static_cast<TMC5240UARTStepper *>(components[id]);
  if (writeLength > 0) {
    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->write_array(data, writeLength);
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
