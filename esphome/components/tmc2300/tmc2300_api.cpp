#include "tmc2300_api_registers.h"
#include "tmc2300_api.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2300 {

uint8_t TMC2300API::crc8_(uint8_t *data, uint32_t bytes) {
  uint8_t result = 0;
  uint8_t *table;

  while (bytes--)
    result = tmc_crc_table_poly7_reflected[result ^ *data++];

  // Flip the result around
  // swap odd and even bits
  result = ((result >> 1) & 0x55) | ((result & 0x55) << 1);
  // swap consecutive pairs
  result = ((result >> 2) & 0x33) | ((result & 0x33) << 2);
  // swap nibbles ...
  result = ((result >> 4) & 0x0F) | ((result & 0x0F) << 4);

  return result;
}

void TMC2300API::set_dirty_bit_(uint8_t index, bool value) {
  if (index >= REGISTER_COUNT)
    return;

  uint8_t *tmp = &this->dirty_bits_[index / 8];
  uint8_t shift = (index % 8);
  uint8_t mask = 1 << shift;
  *tmp = (((*tmp) & (~(mask))) | (((value) << (shift)) & (mask)));
}

bool TMC2300API::get_dirty_bit_(uint8_t index) {
  if (index >= REGISTER_COUNT)
    return false;

  uint8_t *tmp = &this->dirty_bits_[index / 8];
  uint8_t shift = (index % 8);
  return ((*tmp) >> shift) & 1;
}

bool TMC2300API::cache_(CacheOperation operation, uint8_t address, uint32_t *value) {
  if (operation == CACHE_READ) {
    if (IS_READABLE(this->register_access_[address]))
      return false;

    // Grab the value from the cache
    *value = this->shadow_register_[address];
    return true;
  } else if (operation == CACHE_WRITE || operation == CACHE_FILL_DEFAULT) {
    // Fill the cache

    // Write to the shadow register.
    this->shadow_register_[address] = *value;
    // For write operations, mark the register dirty
    if (operation == CACHE_WRITE) {
      this->set_dirty_bit_(address, true);
    }

    return true;
  }
  return false;
}

bool TMC2300API::read_write_register_(uint8_t *data, size_t write_length, size_t read_length) {
  if (write_length > 0) {
    this->write_array(data, write_length);
    this->read_array(data, write_length);
    this->flush();
  }

  optional<bool> ok;
  if (read_length > 0) {
    ok = this->read_array(data, read_length);
  }

  return ok.value_or(false);
}

void TMC2300API::write_register(uint8_t address, int32_t value) {
  ESP_LOGVV(TAG, "writing address 0x%x with value 0x%x (%d)", address, value, value);

  std::array<uint8_t, 8> buffer = {0};

  buffer.at(0) = 0x05;
  buffer.at(1) = this->address_;
  buffer.at(2) = address | TMC_WRITE_BIT;
  buffer.at(3) = (value >> 24) & 0xFF;
  buffer.at(4) = (value >> 16) & 0xFF;
  buffer.at(5) = (value >> 8) & 0xFF;
  buffer.at(6) = (value) &0xFF;
  buffer.at(7) = this->crc8_(buffer.data(), 7);

  this->write_array(buffer.data(), 8);
  this->read_array(buffer.data(), 8);
  this->flush();
  // TODO: maybe do something with IFCNT for write verification

  this->cache_(CACHE_WRITE, address, (uint32_t *) &value);
}

int32_t TMC2300API::read_register(uint8_t address) {
  ESP_LOGVV(TAG, "reading address 0x%x", address);
  uint32_t value;

  // Read from cache for registers with write-only access
  if (this->cache_(CACHE_READ, address, &value))
    return value;

  address = address & TMC_ADDRESS_MASK;
  std::array<uint8_t, 8> buffer = {0};

  buffer.at(0) = 0x05;
  buffer.at(1) = this->address_;
  buffer.at(2) = address;
  buffer.at(3) = this->crc8_(buffer.data(), 3);

  this->write_array(buffer.data(), 4);
  this->read_array(buffer.data(), 4);
  this->flush();

  if (!this->read_array(buffer.data(), 8))
    return 0;

  // Byte 0: Sync nibble correct?
  if (buffer.at(0) != 0x05)
    return 0;

  // Byte 1: Master address correct?
  if (buffer.at(1) != 0xFF)
    return 0;

  // Byte 2: Address correct?
  if (buffer.at(2) != address)
    return 0;

  // Byte 7: CRC correct?
  if (buffer.at(7) != this->crc8_(buffer.data(), 7))
    return 0;

  return encode_uint32(buffer.at(3), buffer.at(4), buffer.at(5), buffer.at(6));
}

uint32_t TMC2300API::update_field(uint32_t data, RegisterField field, uint32_t value) {
  return (data & (~field.mask)) | ((value << field.shift) & field.mask);
}

void TMC2300API::write_field(RegisterField field, uint32_t value) {
  uint32_t reg_value = this->read_register(field.address);
  reg_value = this->update_field(reg_value, field, value);
  this->write_register(field.address, reg_value);
}

uint32_t TMC2300API::extract_field(uint32_t data, RegisterField field) {
  uint32_t value = (data & field.mask) >> field.shift;

  if (field.is_signed) {
    uint32_t base_mask = field.mask >> field.shift;
    uint32_t sign_mask = base_mask & (~base_mask >> 1);
    value = (value ^ sign_mask) - sign_mask;
  }

  return value;
}

uint32_t TMC2300API::read_field(RegisterField field) {
  uint32_t value = this->read_register(field.address);
  return this->extract_field(value, field);
}

}  // namespace tmc2300
}  // namespace esphome
