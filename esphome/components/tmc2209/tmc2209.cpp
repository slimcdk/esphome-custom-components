
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

  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", get_version());

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
  tmc2209_init(&this->driver_, this->channel_, this->address_, &this->config_, &tmc2209_defaultRegisterResetState[0]);
  tmc2209_reset(&this->driver_);

  // Need to disable PDN for UART read
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, 1);

  /*
    // Set toff
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TOFF_MASK, TMC2209_TOFF_SHIFT, 5);
    // Set blank time
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, 0);
    // Set hold current
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, 0);
    // Set run current
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, 16);
    // Set hold current decay delay
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, 15);
    // Set StealthChop
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_PWMCONF, TMC2209_PWM_AUTOSCALE_MASK, TMC2209_PWM_AUTOSCALE_SHIFT, 0);
    TMC2209_FIELD_WRITE(&this->driver_, TMC2209_PWMCONF, TMC2209_PWM_GRAD_MASK, TMC2209_PWM_GRAD_SHIFT, 0);
  */

  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(false);
    this->enable_pin_state_ = false;
  }

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void TMC2209::loop() {
  // if (this->enable_pin_ != nullptr) {
  //   this->enable_pin_->digital_write(true);
  //   this->enable_pin_state_ = true;
  // }
  tmc2209_periodicJob(&this->driver_, 0);  // update the registers
}

int32_t TMC2209::get_version() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
}

}  // namespace tmc
}  // namespace esphome
