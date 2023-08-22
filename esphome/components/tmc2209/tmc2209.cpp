
#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

// TMC-API hardware wrappers
extern "C" void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209 *comp = components[channel];

  comp->write_array(data, writeLength);

  // chop off transmitted bytes from the buffer and flush due to one-wire uart filling up rx when transmitting
  comp->read_array(data, writeLength);
  comp->flush();

  if (readLength) {
    comp->read_array(data, readLength);
  }
}

extern "C" uint8_t tmc2209_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }

void IRAM_ATTR HOT TMC2209IndexStore::gpio_intr(TMC2209IndexStore *arg) {
  if ((arg->target_ - arg->current_) > 0) {
    arg->current_ = arg->current_ + 1;
  } else {
    arg->current_ = arg->current_ - 1;
  }

  if (arg->current_ == arg->target_) {
    arg->target_reached_ = true;
  }
};

void IRAM_ATTR HOT TMC2209DiagStore::gpio_intr(TMC2209DiagStore *arg) {}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  LOG_PIN("  Index Pin: ", this->index_pin_);

  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", this->ioin_chip_version());
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->gconf_microsteps());

  ESP_LOGCONFIG(TAG, "OTP0 Defaults:");
  ESP_LOGCONFIG(TAG, "  TBL:");
  ESP_LOGCONFIG(TAG, "  Internal RSense:");
  ESP_LOGCONFIG(TAG, "  OTTRIM:");
  ESP_LOGCONFIG(TAG, "  FCLKTRIM:");

  ESP_LOGCONFIG(TAG, "OTP1 Defaults:");
  ESP_LOGCONFIG(TAG, "  TPWMTHRS:");
  ESP_LOGCONFIG(TAG, "  CHOPCONF7..5:");
  ESP_LOGCONFIG(TAG, "  CHOPCONF4:");
  ESP_LOGCONFIG(TAG, "  CHOPCONF3..0:");
  ESP_LOGCONFIG(TAG, "  PWM_AUTOGRAD:");
  ESP_LOGCONFIG(TAG, "  PWM_GRAD:");

  ESP_LOGCONFIG(TAG, "OTP2 Defaults:");
  ESP_LOGCONFIG(TAG, "  Spreadcycle:");
  ESP_LOGCONFIG(TAG, "  IHOLD:");
  ESP_LOGCONFIG(TAG, "  IHOLDDELAY:");
  ESP_LOGCONFIG(TAG, "  PWM_FREQ:");
  ESP_LOGCONFIG(TAG, "  PWM_REG:");
  ESP_LOGCONFIG(TAG, "  PWM_OFS:");
  ESP_LOGCONFIG(TAG, "  CHOPCONF8:");

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
  tmc2209_periodicJob(&this->driver_, 0);

  if (this->ioin_chip_version() != TMC2209_DEFAULT_CHIP_VERSION)
    ESP_LOGW(TAG, "Non-default TMC2209 version detected %X. Expected %X", this->ioin_chip_version(),
             TMC2209_DEFAULT_CHIP_VERSION);

  this->enable();

  this->gconf_pdn_disable(1);       // Prioritize UART communication by disabling configuration pin.
  this->gconf_mstep_reg_select(1);  // Use MSTEP register to set microstep resolution
  this->blank_time(0);
  this->gconf_index_otpw(0);
  this->gconf_index_step(1);
  this->gconf_microsteps(5);

  tmc2209_periodicJob(&this->driver_, 0);
  this->disable();

  /*
  // Set toff
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TOFF_MASK, TMC2209_TOFF_SHIFT, 5);

  // Set StealthChop
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_PWMCONF, TMC2209_PWM_AUTOSCALE_MASK, TMC2209_PWM_AUTOSCALE_SHIFT, 0);
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_PWMCONF, TMC2209_PWM_GRAD_MASK, TMC2209_PWM_GRAD_SHIFT, 0);
  */

  // Inttrupt handling for index events
  this->index_pin_->setup();
  this->index_store_.index_pin = this->index_pin_->to_isr();
  this->index_pin_->attach_interrupt(TMC2209IndexStore::gpio_intr, &this->index_store_, gpio::INTERRUPT_RISING_EDGE);

  // Inttrupt handling for diag events
  this->diag_pin_->setup();
  this->diag_store_.diag_pin = this->diag_pin_->to_isr();
  this->diag_pin_->attach_interrupt(TMC2209DiagStore::gpio_intr, &this->diag_store_, gpio::INTERRUPT_ANY_EDGE);

  this->index_store_.current_ = this->current_position;
  this->index_store_.target_ = this->target_position;
  this->index_store_.target_reached_ = this->has_reached_target();

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void TMC2209::loop() {
  if (this->index_store_.target_reached_) {
    this->stop();
    this->index_store_.target_reached_ = true;
    this->index_store_.current_ = this->target_position;
    this->index_store_.target_ = this->target_position;
  }
  this->current_position = this->index_store_.current_;

  // if (this->diag_store_.triggered) {
  //   this->update_driver_status();
  //   this->on_motor_stall_callback_.call();
  // }

  /*
  const uint32_t current_time_ = millis();
  const uint32_t loop_diff_ = current_time_ - this->prev_time_;
  const int32_t position_diff_ = this->prev_position_ - this->current_position;
  this->prev_time_ = current_time_;
  this->prev_position_ = this->current_position;
  ESP_LOGD(TAG, "%.2f", ((float32_t) position_diff_ / (float32_t) loop_diff_));
  */

  tmc2209_periodicJob(&this->driver_, 0);  // update the registers
}

void TMC2209::stop() { this->velocity(0); }

void TMC2209::set_target(int32_t steps) {
  if (this->current_position == steps)
    return;

  this->target_position = steps;
  this->index_store_.target_ = this->target_position;

  const int32_t relative_position = (this->target_position - this->current_position);

  this->index_store_.target_reached_ = false;
  const int32_t velocity = this->max_speed_ * (relative_position < 0 ? 1 : -1);

  this->velocity(velocity);
}

void TMC2209::report_position(int32_t steps) {
  this->index_store_.current_ = steps;
  this->current_position = steps;
}

}  // namespace tmc
}  // namespace esphome
