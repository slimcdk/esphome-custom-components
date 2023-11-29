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
  ESP_LOGCONFIG(TAG, "TMC5240 Stepper:");
  LOG_TMC5240(this);

  // int32_t xactual = TMC5240_FIELD_READ(&this->driver_, TMC5240_XACTUAL, TMC5240_XACTUAL_MASK, TMC5240_XACTUAL_SHIFT);
  // ESP_LOGCONFIG(TAG, "  XACTUAL: 0x%02X", xactual);

  int32_t vmax = TMC5240_FIELD_READ(&this->driver_, TMC5240_VMAX, TMC5240_VMAX_MASK, TMC5240_VMAX_SHIFT);
  ESP_LOGCONFIG(TAG, "  VMAX: 0x%02X", vmax);

  TMC5240_FIELD_WRITE(&this->driver_, TMC5240_VMAX, TMC5240_VMAX_MASK, TMC5240_VMAX_SHIFT, 0x00123456);

  int32_t vmax2 = TMC5240_FIELD_READ(&this->driver_, TMC5240_VMAX, TMC5240_VMAX_MASK, TMC5240_VMAX_SHIFT);
  ESP_LOGCONFIG(TAG, "  VMAX: 0x%02X", vmax2);
}

void TMC5240SPI::read_write(uint8_t *buffer, size_t length) {
  ESP_LOGVV(TAG, "writing=0x%02X %02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
  this->enable();
  this->transfer_array(buffer, length);
  this->disable();
  ESP_LOGVV(TAG, "got back=0x%02X %02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
}

void TMC5240SPI::set_spi_status(uint8_t ss) { this->spi_status_ = ss; }

}  // namespace tmc5240
}  // namespace esphome
