#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/stepper/stepper.h"
#include "esphome/components/uart/uart.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

namespace esphome {
namespace tmc {

#define MAX_ALLOWED_COMPONENTS 3

class TMC2209;  // Forward declare

static TMC2209 *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_global_channel_index = 0;

struct TMC2209IndexStore {
  ISRInternalGPIOPin index_pin;

  volatile int32_t current_{0};
  int32_t target_{0};
  volatile bool target_reached_{true};

  static void gpio_intr(TMC2209IndexStore *arg);
};

struct TMC2209DiagStore {
  ISRInternalGPIOPin diag_pin;
  volatile bool triggered;
  void set_flag(bool high);

  static void gpio_intr(TMC2209DiagStore *arg);
};

class TMC2209 : public stepper::Stepper, public Component, public uart::UARTDevice {
 public:
  TMC2209(uint8_t address, InternalGPIOPin *index_pin, InternalGPIOPin *diag_pin)
      : address_(address), index_pin_(index_pin), diag_pin_(diag_pin) {}

  /* Method overrides */
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;
  void set_target(int32_t steps) override;
  void report_position(int32_t steps) override;

  void enable(bool enable);
  void enable();
  void disable();
  void stop_motion();

  /* Setters */
  void stop_on_fault(bool stop) { this->stop_on_fault_ = stop; };

  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }

  /** CHOPCONF **/

  /** COOLSTEP **/

  /** DRV_STATUS **/

  /** GCONF **/

  /** GSTAT **/

  /** IFCNT **/

  /** IOIN **/

  /** OTP **/

  /** STALLGUARD **/

  /** MISC **/

  void set_inverse_direction(bool inverse);
  void set_velocity(int32_t velocity);
  void set_microsteps(uint8_t ms);
  void set_run_current(int32_t current);
  void set_hold_current(int32_t current);
  void set_hold_current_delay(int32_t current);
  void set_tcool_threshold(int32_t threshold);
  void set_sg_threshold(uint8_t threshold);
  void set_blank_time(uint8_t select);
  void pdn_disable(bool disable);
  void use_mres_register(bool use);

  /* Getters */
  uint8_t get_address() { return this->address_; };
  bool get_ioin_enn_state();
  bool get_ioin_ms1_state();
  bool get_ioin_ms2_state();
  bool get_ioin_diag_state();
  bool get_ioin_pdn_uart_state();
  bool get_ioin_step_state();
  bool get_ioin_spread_en_state();
  bool get_ioin_dir_state();
  bool has_inverse_direction();
  int8_t get_version();
  bool has_reset_since_last_gstat_read();
  bool undervoltage_detection();  // TODO: read continuously and set flag
  uint8_t get_transmission_counter();
  uint16_t get_sg_result();
  uint16_t get_ms_counter();
  int16_t get_ms_current_a();
  int16_t get_ms_current_b();

  uint8_t get_microsteps();

  bool has_driver_error();
  int32_t get_driver_status();

  void add_on_motor_stall_callback(std::function<void()> callback) {
    this->on_motor_stall_callback_.add(std::move(callback));
  }

  TMC2209TypeDef *get_driver() { return &this->driver_; };

 protected:
  /* TMC API stuff */
  uint8_t channel_ = 0;
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  uint8_t address_;
  /* */

  GPIOPin *enable_pin_;
  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;

  TMC2209IndexStore index_store_{};
  TMC2209DiagStore diag_store_{};

  CallbackManager<void()> on_motor_stall_callback_;

  bool is_enabled_{false};
  bool enable_pin_state_;
  bool stop_on_fault_;

  uint32_t sgthrs_;
};

class TMC2209MotorStallTrigger : public Trigger<> {
 public:
  explicit TMC2209MotorStallTrigger(TMC2209 *parent) {
    parent->add_on_motor_stall_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts> class TMC2209ConfigureAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  TEMPLATABLE_VALUE(bool, inverse_direction)
  TEMPLATABLE_VALUE(int, microsteps)
  TEMPLATABLE_VALUE(int, velocity)
  TEMPLATABLE_VALUE(int /*float*/, run_current)
  TEMPLATABLE_VALUE(int /*float*/, hold_current)
  TEMPLATABLE_VALUE(int /*float*/, hold_current_delay)
  TEMPLATABLE_VALUE(int, tcool_threshold)
  TEMPLATABLE_VALUE(int, sg_stall_threshold)

  void play(Ts... x) override {
    TMC2209TypeDef *driver = this->parent_->get_driver();

    // set inverse direction
    if (this->inverse_direction_.has_value())
      this->parent_->set_inverse_direction(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->set_microsteps(this->microsteps_.value(x...));

    if (this->velocity_.has_value())
      this->parent_->set_velocity(this->velocity_.value(x...));

    if (this->run_current_.has_value())
      this->parent_->set_run_current(this->run_current_.value(x...));

    if (this->hold_current_.has_value())
      this->parent_->set_hold_current(this->hold_current_.value(x...));

    if (this->hold_current_delay_.has_value())
      this->parent_->set_hold_current_delay(this->hold_current_delay_.value(x...));

    if (this->tcool_threshold_.has_value())
      this->parent_->set_tcool_threshold(this->tcool_threshold_.value(x...));

    if (this->sg_stall_threshold_.has_value())
      this->parent_->set_sg_threshold(this->sg_stall_threshold_.value(x...));
  }
};

}  // namespace tmc
}  // namespace esphome
