// file: esphome/components/tmc5240_spi/tmc5240_spi.cpp
#include "tmc5240_spi_stepper.h"

#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc5240 {

int32_t TMC5240SPIStepper::read_register(uint8_t address) {
  std::array<uint8_t, 5> data = {
      (address & TMC_ADDRESS_MASK), 0, 0, 0, 0,
  };

  this->enable();
  this->transfer_array(data);
  this->disable();

  // TODO: check data[0] for SPI_STATUS and maybe do something

  return encode_uint32(data[1], data[2], data[3], data[4]);
}

void TMC5240SPIStepper::write_register(uint8_t address, int32_t value) {
  std::array<uint8_t, 5> data = {
      (address | TMC_WRITE_BIT), (0xFF & (value >> 24)), (0xFF & (value >> 16)),
      (0xFF & (value >> 8)),     (0xFF & (value >> 0)),
  };

  this->enable();
  this->transfer_array(data);
  this->disable();
}

}  // namespace tmc5240
}  // namespace esphome
