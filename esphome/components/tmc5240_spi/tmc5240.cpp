#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240_spi {

static const char *TAG = "tmc5240_spi.stepper";

void TMC5240SPIStepper::setup() {
  tmc5240::TMC5240Stepper::setup();
  this->spi_setup();
}

extern "C" {

void tmc5240_readWriteSPI(uint16_t id, uint8_t *data, size_t dataLength) {
  auto *comp = static_cast<TMC5240SPIStepper *>(tmc5240::components[id]);
  comp->enable();
  comp->transfer_array(data, dataLength);
  comp->disable();
}
}

}  // namespace tmc5240_spi
}  // namespace esphome
