
#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

extern "C" {  // TMC-API hardware wrappers
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

void IRAM_ATTR HOT TMC2209IndexStore::gpio_intr(TMC2209IndexStore *arg) {
  if ((arg->target_ - arg->current_) > 0)
    arg->current_ = arg->current_ + 1;
  else
    arg->current_ = arg->current_ - 1;

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

  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", this->read_chip_version());
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->microsteps());
  ESP_LOGCONFIG(TAG, "  Driver Status: %d", this->read_driver_status());
  ESP_LOGCONFIG(TAG, "  Index Output: %d", this->index_step());
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

  bool reset_done = false;
  uint8_t reset_try_count = 10;

  do {
    reset_done = tmc2209_reset(&this->driver_);
  } while (!reset_done && --reset_try_count > 0);
  if (!reset_done)
    this->mark_failed();

  this->enable();

  this->pdn_disable(true);        // Prioritize UART communication by disabling configuration pin.
  this->use_mres_register(true);  // Use MSTEP register to set microstep resolution
  this->blank_time(0);
  this->index_step(true);
  this->microsteps(1);

  this->update_driver_status();  // fetch DRV_STATUS registers
  this->update_ioin();           // fetch IOIN registers

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

  this->disable();

  this->index_store_.current_ = this->current_position;
  this->index_store_.target_ = this->target_position;
  this->index_store_.target_reached_ = this->has_reached_target();
  this->last_mscnt_ = this->internal_step_counter();

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void TMC2209::loop() {
  this->update_driver_status();  // fetch DRV_STATUS registers
  this->update_ioin();           // fetch IOIN registers

  // ESP_LOGD(TAG, "fclktrim=%d ottrim=%d", this->fclktrim(), this->ottrim());

  // if (this->index_store_.target_reached_) {
  //   this->velocity(0);
  //   this->index_store_.target_reached_ = true;
  //   this->index_store_.current_ = this->target_position;
  //   this->index_store_.target_ = this->target_position;
  // }
  // this->current_position = this->index_store_.current_;

  /*if (this->is_enabled_) {
    const uint16_t sg_res = this->stallguard_result();
    const float load = this->calc_motor_load(sg_res);
    // ESP_LOGD(TAG, "%d\t%d\t=\t%.2f", sg_res, this->sg_thrs_, load * 100.0);
    if (load >= 1.0) {
      this->stop();
    }
  }

  if (this->diag_store_.triggered) {
    uint32_t drv_status = this->get_driver_status();
    this->on_motor_stall_callback_.call();
  }

  const uint16_t current_mscnt = this->internal_step_counter();
  const int32_t mscnt_diff = this->last_mscnt_ - current_mscnt;
  this->last_mscnt_ = current_mscnt;

  this->current_position += mscnt_diff;
  */

  // ESP_LOGD(TAG, "%d", this->current_position);
  tmc2209_periodicJob(&this->driver_, 0);  // update the registers
}

void TMC2209::stop() { this->velocity(0); }

void TMC2209::set_target(int32_t steps) {
  // if (this->current_position == steps)
  //   return;

  // this->target_position = steps;
  // this->index_store_.target_ = this->target_position;

  // const int32_t relative_position = (this->target_position - this->current_position);

  // this->index_store_.target_reached_ = false;
  // const int32_t velocity = this->max_speed_ * (relative_position < 0 ? 1 : -1);

  // this->velocity(velocity);
}

void TMC2209::report_position(int32_t steps) {
  this->index_store_.current_ = steps;
  this->current_position = steps;
}

}  // namespace tmc
}  // namespace esphome
