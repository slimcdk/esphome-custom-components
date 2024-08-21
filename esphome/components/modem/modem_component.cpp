#include "modem_component.h"

namespace esphome {
namespace modem {

static const char *TAG = "modem";

void ModemComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Modem...");

  ESP_LOGI(TAG, "Setup done...");
}

void ModemComponent::loop() {}

void ModemComponent::dump_config() { ESP_LOGCONFIG(TAG, "Modem:"); }

}  // namespace modem
}  // namespace esphome
