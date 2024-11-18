
#include "esphome/core/log.h"
#include "icm20948_spi.h"

namespace esphome {
namespace icm20948_spi {

static const char *TAG = "icm20948.spi.sensor";

void ICM20948SPI::setup() { this->spi_setup(); }

void ICM20948SPI::dump_config() {
  ESP_LOGCONFIG(TAG, "ICM20948:");
  LOG_ICM20948(this);
  LOG_PIN("  CS pin: ", this->cs_);
  ESP_LOGCONFIG(TAG, "  Mode: %d", this->mode_);
  if (this->data_rate_ < 1000000) {
    ESP_LOGCONFIG(TAG, "  Data rate: %" PRId32 "kHz", this->data_rate_ / 1000);
  } else {
    ESP_LOGCONFIG(TAG, "  Data rate: %" PRId32 "MHz", this->data_rate_ / 1000000);
  }
}

void ICM20948SPI::on_shutdown() { this->spi_teardown(); };

}  // namespace icm20948_spi
}  // namespace esphome
