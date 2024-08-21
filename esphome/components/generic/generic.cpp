#include "generic.h"
#include "esphome/core/log.h"

namespace esphome {
namespace generic {

#define MCPWM_DUTY_CYCLE 50

static const char *const TAG = "generic.stepper";

void Generic::dump_config() {
  ESP_LOGCONFIG(TAG, "Generic:");
  LOG_PIN("  Step Pin: ", this->step_pin_);
  LOG_PIN("  Dir Pin: ", this->dir_pin_);
  LOG_PIN("  Sleep Pin: ", this->sleep_pin_);
  LOG_STEPPER(this);

  ESP_LOGCONFIG(TAG, "  MCPWM Unit Number: %u", this->mcpwm_unit_);
  ESP_LOGCONFIG(TAG, "  PCNT Unit Number: %u", this->pcnt_unit_);
}

bool Generic::setup_step_counter_() {
  static pcnt_unit_t next_pcnt_unit = PCNT_UNIT_0;
  this->pcnt_unit_ = next_pcnt_unit;
  next_pcnt_unit = pcnt_unit_t(int(next_pcnt_unit) + 1);

  this->pcnt_config_.pulse_gpio_num = this->step_pin_->get_pin();
  this->pcnt_config_.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  this->pcnt_config_.lctrl_mode = PCNT_MODE_KEEP;
  this->pcnt_config_.hctrl_mode = PCNT_MODE_KEEP;
  this->pcnt_config_.pos_mode = PCNT_COUNT_DIS;
  this->pcnt_config_.neg_mode = PCNT_COUNT_DIS;
  this->pcnt_config_.counter_h_lim = 0;
  this->pcnt_config_.counter_l_lim = 0;
  this->pcnt_config_.unit = this->pcnt_unit_;
  this->pcnt_config_.channel = PCNT_CHANNEL_0;

  esp_err_t error = pcnt_unit_config(&this->pcnt_config_);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configuring step counter: %s", esp_err_to_name(error));
    return false;
  }
  error = pcnt_counter_pause(this->pcnt_unit_);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to pausing step counter: %s", esp_err_to_name(error));
    return false;
  }
  error = pcnt_counter_clear(this->pcnt_unit_);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to clear step counter: %s", esp_err_to_name(error));
    return false;
  }
  error = pcnt_counter_resume(this->pcnt_unit_);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to resume step counter: %s", esp_err_to_name(error));
    return false;
  }
  return true;
}

bool Generic::setup_step_generator_() {
  static mcpwm_unit_t next_mcpwm_unit = MCPWM_UNIT_0;
  this->mcpwm_unit_ = next_mcpwm_unit;
  next_mcpwm_unit = mcpwm_unit_t(int(next_mcpwm_unit) + 1);

  this->mcpwm_config_.frequency = (uint32_t) this->max_speed_;
  this->mcpwm_config_.counter_mode = MCPWM_UP_DOWN_COUNTER;
  this->mcpwm_config_.duty_mode = MCPWM_DUTY_MODE_1;
  this->mcpwm_config_.cmpr_a = 50.0;

  esp_err_t error = mcpwm_gpio_init(this->mcpwm_unit_, MCPWM0A, this->step_pin_->get_pin());
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure step generator GPIO: %s", esp_err_to_name(error));
    return false;
  }

  error = mcpwm_init(this->mcpwm_unit_, MCPWM_TIMER_0, &this->mcpwm_config_);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize step generator: %s", esp_err_to_name(error));
    return false;
  }

  error = mcpwm_set_timer_sync_output(this->mcpwm_unit_, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to sync step generator timer: %s", esp_err_to_name(error));
    return false;
  }

  error = mcpwm_stop(this->mcpwm_unit_, MCPWM_TIMER_0);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop step generator: %s", esp_err_to_name(error));
    return false;
  }

  return true;
}

void Generic::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Generic...");
  if (this->sleep_pin_ != nullptr) {
    this->sleep_pin_->setup();
    this->sleep_pin_->digital_write(false);
    this->sleep_pin_state_ = false;
  }
  this->dir_pin_->setup();
  this->dir_pin_->digital_write(false);

  this->step_fb_pin_->setup();
  this->step_fb_pin_->attach_interrupt(Generic::pulse_intr, this, gpio::INTERRUPT_RISING_EDGE);
  // if (!this->setup_step_counter_()) {
  //   this->mark_failed();
  //   return;
  // }

  this->step_pin_->setup();
  this->step_pin_->digital_write(false);
  if (!this->setup_step_generator_()) {
    this->mark_failed();
    return;
  }

  // const auto step_pin_mode = (gpio::FLAG_INPUT | gpio::FLAG_OUTPUT | gpio::FLAG_OPEN_DRAIN);
  // this->step_pin_->pin_mode(step_pin_mode);

  if (this->sleep_pin_ != nullptr) {
    this->sleep_pin_->digital_write(true);
    this->sleep_pin_state_ = true;
  }

  // this->direction = Direction::ANTICLOCKWISE;
  // mcpwm_start(this->mcpwm_unit_, MCPWM_TIMER_0);

  ESP_LOGCONFIG(TAG, "Done setting up Generic...");
}

void IRAM_ATTR HOT Generic::pulse_intr(Generic *arg) {
  arg->current_position += arg->direction;

  time_t now = micros();

  if (arg->has_reached_target()) {
    arg->stop_();
  }
}

void Generic::stop_() {
  this->direction = Direction::NONE;
  mcpwm_stop(this->mcpwm_unit_, MCPWM_TIMER_0);
  mcpwm_set_signal_low(this->mcpwm_unit_, MCPWM_TIMER_0, MCPWM_GEN_A);
  this->high_freq_.stop();
}

void Generic::loop() {
  time_t now = micros();

  this->current_steps_to_target_ = abs(this->current_position - this->target_position);
  const uint32_t steps_traveled = (this->previous_steps_to_target_ - this->current_steps_to_target_);
  const time_t time_passed = abs(now - this->past_);

  this->current_speed_ = ((float) steps_traveled / (float) time_passed) * 1000000.0;
  const float eta = (this->current_steps_to_target_ / this->current_speed_) * 2.0;
  this->eta_ = (eta == eta ? eta : 0);  // avoid nan

  this->average_eta_[this->average_eta_index_] = this->eta_;
  this->average_eta_index_ = (this->average_eta_index_ + 1) % 10;

  float average_eta = 0.0;
  for (uint8_t i = 0; i < 10; i++) {
    average_eta += this->average_eta_[i] / 10.0;

    if (this->average_eta_[i] != this->average_eta_[i]) {
      ESP_LOGW(TAG, "nan eta");
    }

    // / 10.0;
  }

  ESP_LOGI(TAG, "to_target=%8d\ttraveled=%4d\tpassed=%6ld\tspeed=%04.2f\teta=%04.1f", this->current_steps_to_target_,
           steps_traveled, time_passed, this->current_speed_, average_eta);

  this->previous_steps_to_target_ = current_steps_to_target_;
  this->past_ = now;

  this->sleep_pin_->digital_write(this->direction != Direction::NONE);
  this->sleep_pin_state_ = this->direction != Direction::NONE;

  this->dir_pin_->digital_write((this->direction + 1) / 2);

  // ESP_LOGI(TAG, "speed=%f position=%d target=%d direction=%d distance=%d", this->current_speed_,
  // this->current_position,
  //          this->target_position, this->direction, this->current_steps_to_target_);
}

void Generic::set_target(int32_t target) {
  if (this->target_position == target) {
    return;
  }

  this->target_position = target;

  // Compute direction
  const int32_t position_diff = (this->target_position - this->current_position);
  this->direction = (Direction) (position_diff / abs(position_diff));

  this->sleep_pin_->digital_write(this->direction != Direction::NONE);
  this->sleep_pin_state_ = this->direction != Direction::NONE;

  this->high_freq_.start();
  mcpwm_set_duty_type(this->mcpwm_unit_, MCPWM_TIMER_0, MCPWM_GEN_A, this->mcpwm_config_.duty_mode);
  mcpwm_start(this->mcpwm_unit_, MCPWM_TIMER_0);
}

}  // namespace generic
}  // namespace esphome
