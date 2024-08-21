#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/stepper/stepper.h"

#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <driver/pcnt.h>
#include "driver/gpio.h"

namespace esphome {
namespace generic {

enum Direction : int8_t {
  ANTICLOCKWISE = -1,  // Moving one direction
  NONE = 0,            // Not moving
  CLOCKWISE = 1,       // Moving the other direction
};

class Generic : public stepper::Stepper, public Component {
 public:
  void set_step_pin(InternalGPIOPin *step_pin) { this->step_pin_ = step_pin; }
  void set_step_feedback_pin(InternalGPIOPin *step_fb_pin_) { this->step_fb_pin_ = step_fb_pin_; }
  void set_dir_pin(GPIOPin *dir_pin) { this->dir_pin_ = dir_pin; }
  void set_sleep_pin(GPIOPin *sleep_pin) { this->sleep_pin_ = sleep_pin; }
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_target(int32_t target) override;

  Direction direction{Direction::NONE};

 protected:
  static void IRAM_ATTR HOT pulse_intr(Generic *arg);

  bool setup_step_counter_();
  bool setup_step_generator_();
  void stop_();

  uint32_t speed_{0};

  InternalGPIOPin *step_pin_;
  InternalGPIOPin *step_fb_pin_;  // TODO: attatch pcnt to mcpwm gpio
  GPIOPin *dir_pin_;
  GPIOPin *sleep_pin_{nullptr};
  bool sleep_pin_state_;

  mcpwm_config_t mcpwm_config_;
  mcpwm_unit_t mcpwm_unit_;

  pcnt_config_t pcnt_config_;
  pcnt_unit_t pcnt_unit_;

  uint32_t current_steps_to_target_;
  uint32_t previous_steps_to_target_;

  time_t past_{0};
  float eta_{0};

  float average_eta_[10] = {0};
  uint8_t average_eta_index_{0};

  HighFrequencyLoopRequester high_freq_;
  // pcnt_unit_t pcnt_unit;
};

}  // namespace generic
}  // namespace esphome
