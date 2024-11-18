
#include "esphome/core/log.h"
#include "icm20948.h"

namespace esphome {
namespace icm20948 {

static const char *TAG = "icm20948.sensor";

// void ICM20948::setup() {
//   ESP_LOGCONFIG(TAG, "Setting up ICM20948...");
//   ESP_LOGCONFIG(TAG, "ICM20948 is configured!");
// }

uint8_t ICM20948::read_icid() {
  uint8_t val;
  if (!this->read(ICM20X_B0_WHOAMI, &val)) {
    this->mark_failed();
    return {};
  }
  return val;
};

void ICM20948::update() {}

}  // namespace icm20948
}  // namespace esphome
