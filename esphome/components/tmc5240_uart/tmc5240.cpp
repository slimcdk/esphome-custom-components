#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc5240.h"

namespace esphome {
namespace tmc5240_uart {

static const char *TAG = "tmc5240_uart.stepper";

TMC5240UARTStepper::TMC5240UARTStepper(uint8_t address) : address_(address) {}

void TMC5240UARTStepper::setup() { tmc5240::TMC5240Stepper::setup(); }

extern "C" {

uint8_t tmc5240_getNodeAddress(uint16_t id) {
  auto *comp = static_cast<TMC5240UARTStepper *>(tmc5240::components[id]);
  return comp->get_address();
}

bool tmc5240_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
  auto *comp = static_cast<TMC5240UARTStepper *>(tmc5240::components[id]);

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    comp->read_array(data, readLength);
  }
  return true;
}
}

}  // namespace tmc5240_uart
}  // namespace esphome
