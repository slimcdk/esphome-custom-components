#include "tmc2209_hub.h"
#include "esphome/core/helpers.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace tmc2209_hub {

void TMC2209Hub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Hub...");
  ESP_LOGCONFIG(TAG, "TMC2209 Hub setup done.");
}

void TMC2209Hub::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Hub:");
  ESP_LOGCONFIG(TAG, "  Drivers in hub (%d):", this->devices_in_hub_.size());

  for (auto &&device : this->devices_in_hub_) {
    if (device.select_pin_ != nullptr) {
      ESP_LOGCONFIG(TAG, "    Driver with id '%s' on address 0x%02X using select pin: %s", device.id.c_str(),
                    device.address, device.select_pin_->dump_summary().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "    Driver with id '%s' on address 0x%02X", device.id.c_str(), device.address);
    }
  }
}

}  // namespace tmc2209_hub
}  // namespace esphome
