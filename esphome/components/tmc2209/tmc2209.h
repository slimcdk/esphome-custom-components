#pragma once

#include "esphome/core/helpers.h"

#include "esphome/components/uart/uart.h"

extern "C" {
#include <ic/TMC2209/TMC2209.h>
}

namespace esphome {
namespace tmc {

#define MAX_ALLOWED_COMPONENTS 3
#define TMC2209_DEFAULT_CHIP_VERSION 0x21

class TMC2209;  // Forward declare

static TMC2209 *components[MAX_ALLOWED_COMPONENTS];
static uint8_t tmc2209_global_channel_index = 0;

struct TMC2209DiagStore {
  ISRInternalGPIOPin diag_pin;
  volatile bool triggered;
  void set_flag(bool high);
  static void gpio_intr(TMC2209DiagStore *arg);
};

class TMC2209 : public uart::UARTDevice {
 public:
  TMC2209();

  void blank_time(uint8_t select);
  void tcool_threshold(int32_t threshold);
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
  uint16_t gstat();
  void gstat(uint16_t setting);
  bool gstat_reset();
  void gstat_reset(bool clear);
  bool gstat_drv_err();
  void gstat_drv_err(bool clear);
  bool gstat_uv_cp();
  void gstat_uv_cp(bool clear);
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
  uint8_t transmission_counter();
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
  uint32_t otpread();
  bool optread_en_spreadcycle();
  void fclktrim(uint8_t fclktrim);
  uint8_t fclktrim();
  void ottrim(uint8_t ottrim);
  uint8_t ottrim();
  // void stallguard_threshold(uint8_t threshold);
  uint16_t stallguard_result();
  // float calculate_motor_load(uint16_t sg_result);
  uint16_t internal_step_counter();  // Difference since last poll. Wrap around at 1023
  int16_t current_a();
  int16_t current_b();
  void velocity(int32_t velocity);
  void ihold_irun_ihold(int32_t current);
  void ihold_irun_irun(int32_t current);
  void ihold_irun_ihold_delay(int32_t current);

  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }
  void set_address(uint8_t address) { this->address_ = address; }

  void enable(bool enable = true);
  void disable();

  TMC2209TypeDef *get_driver() { return &this->driver_; };
  uint8_t get_address() { return this->address_; };

 protected:
  uint8_t channel_ = 0;
  TMC2209TypeDef driver_;
  ConfigurationTypeDef config_;
  uint8_t address_;

  bool is_enabled_{false};
  bool enable_pin_state_;

  GPIOPin *enable_pin_;
  InternalGPIOPin *index_pin_;
  InternalGPIOPin *diag_pin_;
  TMC2209DiagStore diag_store_{};

  void tmc2209_setup();
  void tmc2209_post_setup();
  void tmc2209_loop();
};

}  // namespace tmc
}  // namespace esphome
