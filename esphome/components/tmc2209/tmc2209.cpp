
#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

extern "C" {
void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  // ESP_LOGD(TAG, "tmc2209_readWriteArray(%d, %d, %d, %d)", channel, *data, writeLength, readLength);

  comp->write_array(data, writeLength);
  comp->flush();  // flush due to one-wire uart filling up rx when transmitting
  if (readLength) {
    if (!comp->read_array(data, readLength)) {
      ESP_LOGE(TAG, "Error reading uart");
    };
  }
}

uint8_t tmc2209_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }
}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");
  LOG_PIN("  Step Pin: ", this->step_pin_);
  LOG_PIN("  Dir Pin: ", this->dir_pin_);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_STEPPER(this);
}

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  comp = this;

  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(false);
    this->enable_pin_state_ = false;
  }
  this->step_pin_->setup();
  this->step_pin_->digital_write(false);
  this->dir_pin_->setup();
  this->dir_pin_->digital_write(false);

  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2209_init(this->driver, this->channel_, this->slaveAddress_, this->driver_config,
               &tmc2209_defaultRegisterResetState[0]);
  tmc2209_reset(this->driver);

  // Need to disable PDN for UART read
  TMC2209_FIELD_WRITE(this->driver, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, 1);
  // Set toff
  TMC2209_FIELD_WRITE(this->driver, TMC2209_CHOPCONF, TMC2209_TOFF_MASK, TMC2209_TOFF_SHIFT, 5);
  // Set microstepping
  TMC2209_FIELD_WRITE(this->driver, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, 0);
  // Set blank time
  TMC2209_FIELD_WRITE(this->driver, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, 0);
  // Set hold current
  TMC2209_FIELD_WRITE(this->driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, 0);
  // Set run current
  TMC2209_FIELD_WRITE(this->driver, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, 16);
  // Set hold current decay delay
  TMC2209_FIELD_WRITE(this->driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, 15);
  // Set StealthChop
  TMC2209_FIELD_WRITE(this->driver, TMC2209_PWMCONF, TMC2209_PWM_AUTOSCALE_MASK, TMC2209_PWM_AUTOSCALE_SHIFT, 1);
  TMC2209_FIELD_WRITE(this->driver, TMC2209_PWMCONF, TMC2209_PWM_GRAD_MASK, TMC2209_PWM_GRAD_SHIFT, 1);

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void TMC2209::loop() {
  tmc2209_periodicJob(this->driver, 0);  // update the registers

  if (this->last_run_ + 5000 < millis()) {
    this->last_run_ = millis();

    // Change direction
    this->direction = !this->direction;
    TMC2209_FIELD_WRITE(this->driver, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, this->direction);

    // Set motor velocity
    TMC2209_FIELD_WRITE(this->driver, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, 1000);

    // int32_t gconf_status =
    //     TMC2209_FIELD_READ(this->driver, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT);
    // ESP_LOGI(TAG, "gconf_status: %d", gconf_status);

    int32_t driver_version_status =
        TMC2209_FIELD_READ(this->driver, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
    ESP_LOGI(TAG, "driver_version_status: %d", driver_version_status);
  }

  bool at_target = this->has_reached_target();
  if (this->enable_pin_ != nullptr) {
    bool sleep_rising_edge = !enable_pin_state_ & !at_target;
    this->enable_pin_->digital_write(!at_target);
    this->enable_pin_state_ = !at_target;
    if (sleep_rising_edge) {
      delayMicroseconds(1000);
    }
  }
  if (at_target) {
    this->high_freq_.stop();
  } else {
    this->high_freq_.start();
  }

  int32_t dir = this->should_step_();
  if (dir == 0)
    return;

  this->dir_pin_->digital_write(dir == 1);
  this->step_pin_->digital_write(true);
  delayMicroseconds(5);
  this->step_pin_->digital_write(false);
}

}  // namespace tmc
}  // namespace esphome
