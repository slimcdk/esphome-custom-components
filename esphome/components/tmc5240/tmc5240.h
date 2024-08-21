#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/stepper/stepper.h"

extern "C" {
#include <ic/TMC5240/TMC5240.h>
}

namespace esphome {
namespace tmc5240 {

#define LOG_TMC5240(this) \
  LOG_PIN("  Enn Pin: ", this->enn_pin_); \
  LOG_PIN("  DIAG0 Pin: ", this->diag0_pin_); \
  LOG_PIN("  DIAG1 Pin: ", this->diag1_pin_); \
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X (revision: 0x%02X)", this->get_version(), this->get_revision()); \
  ESP_LOGCONFIG(TAG, "  External oscillator: %s", this->get_using_external_oscillator() ? "True" : "False"); \
  ESP_LOGCONFIG(TAG, "  Encoder latched: %d", this->enable_encoder_position()); \
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps()); \
  if (this->get_adc_err()) { \
    ESP_LOGE(TAG, "  Faulty ADC detected! Advised not to use ADC type input!"); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Supply Voltage: %.0f mV", this->get_vsupply()); \
  }

class TMC5240 : public Component, public stepper::Stepper {
 public:
  TMC5240();

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;

  void set_enn_pin(GPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag0_pin(GPIOPin *pin) { this->diag0_pin_ = pin; }
  void set_diag1_pin(GPIOPin *pin) { this->diag1_pin_ = pin; }

  float get_vsupply();
  float get_temp();

  int32_t get_xactual();
  void set_xactual(int32_t value);
  int32_t get_vactual();

  void set_vmax(int32_t max);
  void set_amax(int32_t max);
  void set_dmax(int32_t max);
  void set_tvmax(int32_t max);

  int32_t get_sg4_result();
  virtual void read_write(uint8_t *buffer, size_t length) = 0;  // make tmc-api friends of TMC5240 and protect this

  void set_enc_const(float value);
  float get_enc_const();
  int32_t get_x_enc();
  uint8_t get_encmode_ignore_ab();
  uint8_t get_version();
  uint8_t get_revision();
  bool get_using_external_oscillator();
  bool get_adc_err();

  bool get_shaft_direction();
  void set_shaft_direction(bool invert);

  void enable_encoder_position(bool enable);
  bool enable_encoder_position();

  void chopconf_mres(uint8_t index);
  uint8_t chopconf_mres();

  uint32_t enc_deviation();
  void enc_deviation(uint32_t deviation);

  uint16_t get_microsteps();
  void set_microsteps(uint16_t ms);

  void add_on_stall_callback(std::function<void()> callback) { this->on_stall_callback_.add(std::move(callback)); }

  bool get_gconf_diag0_int_pushpull();

  // void rms_current(float A);
  // float rms_current();
  // void rms_current_hold_scale(float scale);
  // float rms_current_hold_scale();
  // void set_rms_current_();
  // float current_scale_to_rms_current_(uint8_t current_scaling);

 protected:
  GPIOPin *enn_pin_;
  GPIOPin *diag0_pin_;
  GPIOPin *diag1_pin_;

  // TMC-API handlers
  uint8_t comp_index_{0};  // used for tmcapi channel index and esphome global component index
  TMC5240TypeDef driver_;
  ConfigurationTypeDef config_;
  void update_registers_();
  bool reset_();
  bool restore_();

  float rms_current_{std::numeric_limits<float>::max()};
  float rms_current_hold_scale_{1.0};

  CallbackManager<void()> on_stall_callback_;
};

/**
 * Components are globally indexed such that TMC-API functions can access the objects/components.
 */
static TMC5240 *components[TMC5240_NUM_COMPONENTS];
static uint8_t tmc5240_global_component_index = 0;

template<typename... Ts> class TMC5240ConfigureAction : public Action<Ts...>, public Parented<TMC5240> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(float, rms_current)
  TEMPLATABLE_VALUE(int, coolstep_tcoolthrs)
  TEMPLATABLE_VALUE(int, stallguard_sgthrs)

  void play(Ts... x) override {
    if (this->inverse_direction_.has_value())
      this->parent_->set_shaft_direction(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    // if (this->rms_current_.has_value())
    //   this->parent_->rms_current(this->rms_current_.value(x...));

    // if (this->coolstep_tcoolthrs_.has_value())
    //   this->parent_->coolstep_tcoolthrs(this->coolstep_tcoolthrs_.value(x...));

    // if (this->stallguard_sgthrs_.has_value())
    //   this->parent_->stallguard_sgthrs(this->stallguard_sgthrs_.value(x...));
  }
};

template<typename... Ts> class TMC5240StopAction : public Action<Ts...>, public Parented<TMC5240> {
 public:
  void play(Ts... x) override {
    // TODO: this->parent_->stop();
  }
};

class TMC5240OnStallTrigger : public Trigger<> {
 public:
  explicit TMC5240OnStallTrigger(TMC5240 *parent) {
    parent->add_on_stall_callback([this]() { this->trigger(); });
  }
};

}  // namespace tmc5240
}  // namespace esphome
