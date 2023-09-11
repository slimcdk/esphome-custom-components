
#include "esphome/core/log.h"
#include "tmc2209_stepper.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

void IRAM_ATTR HOT TMC2209StepperIndexStore::gpio_intr(TMC2209StepperIndexStore *arg) {
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

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  LOG_PIN("  Index Pin: ", this->index_pin_);

  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", this->ioin_chip_version());
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->gconf_microsteps());
  LOG_STEPPER(this);
}

void TMC2209Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  // Inttrupt handling for index events
  this->index_pin_->setup();
  this->index_store_.index_pin = this->index_pin_->to_isr();
  this->index_pin_->attach_interrupt(TMC2209StepperIndexStore::gpio_intr, &this->index_store_,
                                     gpio::INTERRUPT_RISING_EDGE);
  this->index_store_.current_ = this->current_position;
  this->index_store_.target_ = this->target_position;
  this->index_store_.target_reached_ = this->has_reached_target();

  this->enable();
  this->tmc2209_setup();

  this->gconf_mstep_reg_select(1);  // Use MSTEP register to set microstep resolution
  this->blank_time(0);
  this->gconf_index_otpw(0);
  this->gconf_index_step(1);
  this->gconf_microsteps(5);

  this->tmc2209_post_setup();
  this->disable();

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::loop() {
  this->tmc2209_loop();

  if (this->index_store_.target_reached_) {
    this->stop();
    this->index_store_.target_reached_ = true;
    this->index_store_.current_ = this->target_position;
    this->index_store_.target_ = this->target_position;
  }
  this->current_position = this->index_store_.current_;
}

void TMC2209Stepper::stop() { this->velocity(0); }

void TMC2209Stepper::set_target(int32_t steps) {
  if (this->current_position == steps)
    return;

  this->target_position = steps;
  this->index_store_.target_ = this->target_position;

  const int32_t relative_position = (this->target_position - this->current_position);

  this->index_store_.target_reached_ = false;
  const int32_t velocity = this->max_speed_ * (relative_position < 0 ? 1 : -1);

  this->velocity(velocity);
}

void TMC2209Stepper::report_position(int32_t steps) {
  this->index_store_.current_ = steps;
  this->current_position = steps;
}

}  // namespace tmc
}  // namespace esphome
