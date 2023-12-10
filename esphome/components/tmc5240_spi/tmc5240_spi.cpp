#include "tmc5240_spi.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc5240 {

static const char *TAG = "tmc5240_spi";

void TMC5240SPI::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC5240...");

  this->spi_setup();
  while (!this->spi_is_ready()) {
  }

  TMC5240::setup();
  ESP_LOGCONFIG(TAG, "TMC5240 Stepper setup done.");
}

void TMC5240SPI::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC5240 SPI Stepper:");
  LOG_TMC5240(this);
}

void TMC5240SPI::read_write(uint8_t *buffer, size_t length) {
  this->enable();
  this->transfer_array(buffer, length);
  this->disable();
}

void TMC5240SPI::set_spi_status(uint8_t ss) { this->spi_status_ = ss; }

}  // namespace tmc5240
}  // namespace esphome
