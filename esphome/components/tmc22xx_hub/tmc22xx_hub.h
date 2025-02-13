#pragma once

#include <vector>

#include "esphome/components/uart/uart.h"

namespace esphome {
namespace tmc22xx_hub {

static const char *const TAG = "tmc22xx_hub";

struct HubDevice {
  std::string id;
  uint8_t address;
};

class TMC22XXHub : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::BUS; }
  void setup() override;
  void dump_config() override;

  void notify_device_in_hub_(std::string id, uint8_t address) { this->devices_in_hub_.push_back({id, address}); }

 protected:
  std::vector<HubDevice> devices_in_hub_;
};

}  // namespace tmc22xx_hub
}  // namespace esphome
