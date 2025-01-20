#pragma once

#include "esphome/components/uart/uart.h"

namespace esphome {
namespace tmc2209_hub {

class TMC2209Hub : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
};

class TMC2209Device {
 public:
  TMC2209Device() = default;
  TMC2209Device(TMC2209Hub *parent) : parent_(parent) {}

  void set_tmc2209_hub_parent(TMC2209Hub *parent) { this->parent_ = parent; }

 protected:
  TMC2209Hub *parent_{nullptr};
};

}  // namespace tmc2209_hub
}  // namespace esphome
