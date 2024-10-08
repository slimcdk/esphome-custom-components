// file: esphome/components/tmc5241_spi/tmc5241_spi.cpp
#include "tmc5241_spi_stepper.h"

#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc5241 {

int32_t TMC5241SPIStepper::read_register(uint8_t address) {
  std::array<uint8_t, 5> data = {0};

  data[0] = address & TMC_ADDRESS_MASK;

  this->enable();
  this->transfer_array(data);
  this->disable();

  // TODO: check data[0] for SPI_STATUS and maybe do something

  return encode_uint32(data[1], data[2], data[3], data[4]);
}

void TMC5241SPIStepper::write_register(uint8_t address, int32_t value) {
  std::array<uint8_t, 5> data = {0};

  data[0] = address | TMC_WRITE_BIT;
  data[1] = 0xFF & (value >> 24);
  data[2] = 0xFF & (value >> 16);
  data[3] = 0xFF & (value >> 8);
  data[4] = 0xFF & (value >> 0);

  this->enable();
  this->transfer_array(data);
  this->disable();
}

}  // namespace tmc5241
}  // namespace esphome
