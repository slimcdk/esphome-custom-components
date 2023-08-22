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

#define TMC2209_DEFAULT_CHIP_VERSION 0x21
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

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void loop() override;

  void enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }

  void set_target(int32_t steps) override;
  void report_position(int32_t steps) override;

  void enable(bool enable = true);
  void disable();
  void stop();

  /** CHOPCONF **/
  void blank_time(uint8_t select);

  /** COOLCONF **/
  void tcool_threshold(int32_t threshold);

  /** GCONF **/
  uint16_t gconf();
  void gconf(uint16_t setting);
  void gconf_iscale_analog(bool use_vref);
  bool gconf_iscale_analog();
  void gconf_internal_rsense(bool use_internal);
  bool gconf_internal_rsense();
  void gconf_en_spreadcycle(bool enable);
  bool gconf_en_spreadcycle();
  bool gconf_inverse_direction();
  void gconf_inverse_direction(bool inverse_direction);
  bool gconf_index_otpw();
  void gconf_index_otpw(bool use_otpw);
  void gconf_index_step(bool enable);
  bool gconf_index_step();
  void gconf_pdn_disable(bool disable);
  bool gconf_pdn_disable();
  void gconf_mstep_reg_select(bool use);
  bool gconf_mstep_reg_select();
  void gconf_microsteps(uint8_t ms);
  uint8_t gconf_microsteps();
  void gconf_multistep_filt(bool enable);
  bool gconf_multistep_filt();
  void gconf_test_mode(bool enable);
  bool gconf_test_mode();

  /** GSTAT **/
  uint16_t gstat();
  void gstat(uint16_t setting);
  bool gstat_reset();
  void gstat_reset(bool clear);
  bool gstat_drv_err();
  void gstat_drv_err(bool clear);
  bool gstat_uv_cp();
  void gstat_uv_cp(bool clear);

  /** DRV_STATUS **/
  bool has_driver_error();
  uint32_t driver_status();
  bool drv_status_stst();
  bool drv_status_stealth();
  uint8_t drv_status_cs_actual();
  bool drv_status_otpw();
  bool drv_status_ot();
  bool drv_status_t120();
  bool drv_status_t143();
  bool drv_status_t150();
  bool drv_status_t157();
  bool drv_status_ola();
  bool drv_status_olb();
  bool drv_status_s2vsa();
  bool drv_status_s2vsb();
  bool drv_status_s2ga();
  bool drv_status_s2gb();

  /** IFCNT **/
  uint8_t transmission_counter();

  /** IOIN **/
  uint32_t ioin();
  bool ioin_enn();
  bool ioin_ms1();
  bool ioin_ms2();
  bool ioin_diag();
  bool ioin_pdn_uart();
  bool ioin_step();
  bool ioin_spread_en();
  bool ioin_dir();
  int8_t ioin_chip_version();

  /** OTP **/
  uint32_t otpread();
  bool optread_en_spreadcycle();

  /** FACTORY CONF **/
  void fclktrim(uint8_t fclktrim);
  uint8_t fclktrim();
  void ottrim(uint8_t ottrim);
  uint8_t ottrim();

  /** STALLGUARD **/
  void stallguard_threshold(uint8_t threshold);
  uint16_t stallguard_result();
  float calculate_motor_load(uint16_t sg_result);

  /** DRV_CTRL **/
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  int16_t current_a();
  int16_t current_b();
  void velocity(int32_t velocity);
  void ihold_irun_ihold(int32_t current);
  void ihold_irun_irun(int32_t current);
  void ihold_irun_ihold_delay(int32_t current);

  TMC2209TypeDef *get_driver() { return &this->driver_; };
  uint8_t address() { return this->address_; };

  void add_on_motor_stall_callback(std::function<void()> callback) {
    this->on_motor_stall_callback_.add(std::move(callback));
  }

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

  uint32_t prev_time_;
  uint32_t prev_position_{0};
  uint32_t sg_thrs_;
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
  TEMPLATABLE_VALUE(int, run_current)
  TEMPLATABLE_VALUE(int, hold_current)
  TEMPLATABLE_VALUE(int, hold_current_delay)
  TEMPLATABLE_VALUE(int, tcool_threshold)
  TEMPLATABLE_VALUE(int, stallguard_threshold)

  void play(Ts... x) override {
    TMC2209TypeDef *driver = this->parent_->get_driver();

    // set inverse direction
    if (this->inverse_direction_.has_value())
      this->parent_->gconf_inverse_direction(this->inverse_direction_.value(x...));

    if (this->microsteps_.has_value())
      this->parent_->gconf_microsteps(this->microsteps_.value(x...));

    if (this->velocity_.has_value())
      this->parent_->velocity(this->velocity_.value(x...));

    if (this->run_current_.has_value())
      this->parent_->ihold_irun_irun(this->run_current_.value(x...));

    if (this->hold_current_.has_value())
      this->parent_->ihold_irun_ihold(this->hold_current_.value(x...));

    if (this->hold_current_delay_.has_value())
      this->parent_->ihold_irun_ihold_delay(this->hold_current_delay_.value(x...));

    if (this->tcool_threshold_.has_value())
      this->parent_->tcool_threshold(this->tcool_threshold_.value(x...));

    if (this->stallguard_threshold_.has_value())
      this->parent_->stallguard_threshold(this->stallguard_threshold_.value(x...));
  }
};

template<typename... Ts> class TMC2209StopAction : public Action<Ts...>, public Parented<TMC2209> {
 public:
  void play(Ts... x) override {
    TMC2209TypeDef *driver = this->parent_->get_driver();
    this->parent_->stop();
  }
};

}  // namespace tmc
}  // namespace esphome
