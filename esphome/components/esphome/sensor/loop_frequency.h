#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace loop_frequency {

static const char *const TAG = "loop_frequency.sensor";

class LoopFrequencySensor : public PollingComponent, public sensor::Sensor {
 public:
  void dump_config() override { LOG_SENSOR(" ", "Loop Frequency Sensor", this); }

  void setup() override { this->high_freq_.start(); }

  void update() override { this->publish_state(this->diff_ == 0 ? 0 : (1000000 / this->diff_)); };
  // void update() override { this->publish_state(this->diff_); };

  void loop() override {
    const unsigned long now = micros();
    this->diff_ = (now - this->last_now_);
    this->last_now_ = now;
  };

 protected:
  unsigned long last_now_{0};
  unsigned long diff_{0};

  HighFrequencyLoopRequester high_freq_;
};

}  // namespace loop_frequency
}  // namespace esphome
