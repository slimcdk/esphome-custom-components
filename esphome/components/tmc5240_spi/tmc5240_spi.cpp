#include "tmc5240_spi.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc5240 {

static const char *TAG = "tmc5240_spi";

// #define DECODE_SS(tag, ss)

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
  ESP_LOGVV(TAG, "writing=0x%02X %02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
  this->enable();
  this->transfer_array(buffer, length);
  this->disable();
  ESP_LOGVV(TAG, "got back=0x%02X %02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

  ESP_LOGV(TAG, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(buffer[0]));
}

void TMC5240SPI::set_spi_status(uint8_t ss) { this->spi_status_ = ss; }

}  // namespace tmc5240
}  // namespace esphome
