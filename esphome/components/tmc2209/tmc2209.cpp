
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

extern "C" {
void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209 *comp = components[channel];

  comp->write_array(data, writeLength);

  // chop off transmitted bytes from the buffer and flush due to one-wire uart filling up rx when transmitting
  comp->read_array(data, writeLength);
  comp->flush();

  if (readLength) {
    comp->read_array(data, readLength);
  }
}

uint8_t tmc2209_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }
}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  LOG_PIN("  Index Pin: ", this->index_pin_);

  int32_t driver_version = TMC2209_FIELD_READ(&this->driver, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", driver_version);

  if (this->version_text_sensor_ != nullptr) {
    LOG_TEXT_SENSOR("  ", "Version Text Sensor", this->version_text_sensor_);
  }

  LOG_STEPPER(this);
}

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  this->channel_ = ++tmc2209_global_channel_index;
  components[this->channel_] = this;

  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(true);
    this->enable_pin_state_ = true;
  }

  // Initialize TMC driver instance
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2209_init(&this->driver, this->channel_, this->address_, &this->config, &tmc2209_defaultRegisterResetState[0]);
  tmc2209_reset(&this->driver);

  // Need to disable PDN for UART read
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, 1);

  // Set toff
  /*TMC2209_FIELD_WRITE(&this->driver, TMC2209_CHOPCONF, TMC2209_TOFF_MASK, TMC2209_TOFF_SHIFT, 5);
  // Set blank time
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, 0);
  // Set hold current
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, 0);
  // Set run current
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, 16);
  // Set hold current decay delay
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, 15);
  // Set StealthChop
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_PWMCONF, TMC2209_PWM_AUTOSCALE_MASK, TMC2209_PWM_AUTOSCALE_SHIFT, 0);
  TMC2209_FIELD_WRITE(&this->driver, TMC2209_PWMCONF, TMC2209_PWM_GRAD_MASK, TMC2209_PWM_GRAD_SHIFT, 0);
  */
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(false);
    this->enable_pin_state_ = false;
  }

  int32_t driver_version = TMC2209_FIELD_READ(&this->driver, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
  ESP_LOGD(TAG, "driver version: %d (0x%02X)", driver_version, driver_version);

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void TMC2209::loop() {
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(true);
    this->enable_pin_state_ = true;
  }

  tmc2209_periodicJob(&this->driver, 0);  // update the registers
}

void TMC2209::update() {
  if (this->version_text_sensor_ != nullptr) {
    const int32_t driver_version =
        TMC2209_FIELD_READ(&this->driver, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
    this->version_text_sensor_->publish_state(str_sprintf("0x%02X", driver_version));

    ESP_LOGD(TAG, "driver version: %d (0x%02X)", driver_version, driver_version);
  }
}

}  // namespace tmc
}  // namespace esphome
