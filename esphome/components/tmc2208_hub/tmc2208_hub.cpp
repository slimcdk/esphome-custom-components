#include "tmc2208_hub.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2208_hub {

void TMC2208Hub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2208 Hub...");
  ESP_LOGCONFIG(TAG, "TMC2208 Hub setup done.");
}

void TMC2208Hub::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2208 Hub:");
  ESP_LOGCONFIG(TAG, "  Drivers in hub (%d):", this->devices_in_hub_.size());

  for (auto &&device : this->devices_in_hub_) {
    ESP_LOGCONFIG(TAG, "    Driver with id '%s' on address 0x%02X", device.id.c_str(), device.address);
  }
}

}  // namespace tmc2208_hub
}  // namespace esphome
