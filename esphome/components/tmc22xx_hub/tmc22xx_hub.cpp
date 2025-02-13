#include "tmc22xx_hub.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc22xx_hub {

void TMC22XXHub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC22XX Hub...");
  ESP_LOGCONFIG(TAG, "TMC22XX Hub setup done.");
}

void TMC22XXHub::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Hub:");
  ESP_LOGCONFIG(TAG, "  Drivers in hub (%d):", this->devices_in_hub_.size());

  for (auto &&device : this->devices_in_hub_) {
    ESP_LOGCONFIG(TAG, "    Driver with id '%s' on address 0x%02X", device.id.c_str(), device.address);
  }
}

}  // namespace tmc22xx_hub
}  // namespace esphome
