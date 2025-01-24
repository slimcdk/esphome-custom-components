#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209_hub {

static const char *const TAG = "tmc2209_hub";

class TMC2209Hub : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() { ESP_LOGCONFIG(TAG, "TMC2209 Hub:"); }
};

class TMC2209HubDevice : public Parented<TMC2209Hub> {
 public:
  TMC2209HubDevice() = default;
  TMC2209HubDevice(TMC2209Hub *parent) : parent_(parent) {}

  void set_tmc2209_hub_parent(TMC2209Hub *parent) { this->parent_ = parent; }

 protected:
  TMC2209Hub *parent_{nullptr};
};

}  // namespace tmc2209_hub
}  // namespace esphome
