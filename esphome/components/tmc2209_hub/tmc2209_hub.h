#pragma once

#include <vector>

#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace tmc2209_hub {

static const char *const TAG = "tmc2209_hub";

struct HubDevice {
  std::string id;
  uint8_t address;
};

class TMC2209Hub : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::BUS; }
  void setup() override;
  void dump_config() override;

  void add_device_to_hub_(std::string id, uint8_t address) { this->devices_in_hub_.push_back({id, address}); }

 protected:
  std::vector<HubDevice> devices_in_hub_;
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
