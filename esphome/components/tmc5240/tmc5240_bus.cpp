// #pragma once

// #include "esphome/core/helpers.h"
// #include "esphome/core/component.h"
// #include "esphome/core/hal.h"

// #include "esphome/components/stepper/stepper.h"
// #include "esphome/components/spi/spi.h"
// #include "esphome/components/uart/uart.h"

// extern "C" {
// #include <ic/TMC5240/TMC5240.h>
// #include <helpers/Bits.h>
// #include <helpers/Macros.h>
// }

// namespace esphome {
// namespace tmc5240 {

// extern "C" {
// uint8_t tmc5240_getBusType(uint16_t id) {
//   auto *comp = components[id];
//   return comp->get_bus_type();
// }
// }

// #if defined TMC5240_USE_SPI
// extern "C" {

// void tmc5240_readWriteSPI(uint16_t id, uint8_t *data, size_t dataLength) {
//   auto *comp = static_cast<TMC5240SPIStepper *>(tmc5240::components[id]);

//   if (comp == nullptr) {
//     ESP_LOGE(TAG, "Component with id %d is null", id);
//     return;
//   }

//   comp->enable();
//   comp->transfer_array(data, dataLength);
//   comp->disable();
// }

// uint8_t tmc5240_getNodeAddress(uint16_t id) {}
// bool tmc5240_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {}
// }
// #endif

// #if defined TMC5240_USE_UART
// extern "C" {

// uint8_t tmc5240_getNodeAddress(uint16_t id) {
//   auto *comp = static_cast<TMC5240Bus *>(tmc5240::components[id]);
//   return comp->get_address();
// }

// bool tmc5240_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
//   auto *comp = static_cast<TMC5240Bus *>(tmc5240::components[id]);

//   if (comp == nullptr) {
//     ESP_LOGE(TAG, "Component with id %d is null", id);
//     return false;
//   }

//   if (writeLength > 0) {
//     comp->write_array(data, writeLength);

//     // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
//     comp->read_array(data, writeLength);
//     comp->flush();
//   }

//   if (readLength > 0) {
//     comp->read_array(data, readLength);
//   }
//   return true;
// }
// }

// void tmc5240_readWriteSPI(uint16_t id, uint8_t *data, size_t dataLength);
// #endif

// }  // namespace tmc5240
// }  // namespace esphome
