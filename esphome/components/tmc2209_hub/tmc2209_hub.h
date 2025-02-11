#pragma once

#include <vector>

#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace tmc2209_hub {

static const char *const TAG = "tmc2209_hub";

struct HubDevice {
  const std::string id;
  const uint8_t address;
  const GPIOPin *select_pin_;
};

class TMC2209Hub : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::BUS; }
  void setup() override;
  void dump_config() override;

  void notify_device_in_hub_(std::string id, uint8_t address, GPIOPin *sel_pin = nullptr) {
    this->devices_in_hub_.push_back({id, address, sel_pin});
  }

 protected:
  std::vector<HubDevice> devices_in_hub_;
};

}  // namespace tmc2209_hub
}  // namespace esphome
