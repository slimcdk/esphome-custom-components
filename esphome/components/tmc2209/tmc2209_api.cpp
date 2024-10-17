#include "tmc2209_api.h"

namespace esphome {
namespace tmc2209 {

uint8_t TMC2209API::crc8_(uint8_t *data, uint32_t bytes) {
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

void TMC2209API::set_dirty_bit_(uint8_t index, bool value) {
  if (index >= REGISTER_COUNT)
    return;

  uint8_t *tmp = &this->dirty_bits_[index / 8];
  uint8_t shift = (index % 8);
  uint8_t mask = 1 << shift;
  *tmp = (((*tmp) & (~(mask))) | (((value) << (shift)) & (mask)));
}

bool TMC2209API::get_dirty_bit_(uint8_t index) {
  if (index >= REGISTER_COUNT)
    return false;

  uint8_t *tmp = &this->dirty_bits_[index / 8];
  uint8_t shift = (index % 8);
  return ((*tmp) >> shift) & 1;
}

bool TMC2209API::cache_(CacheOperation operation, uint8_t address, uint32_t *value) {
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

bool TMC2209API::read_write_register_(uint8_t *data, size_t write_length, size_t read_length) {
  if (write_length > 0) {
    this->write_array(data, write_length);
    this->read_array(data, write_length);
    this->flush();
    // TODO: maybe do something with IFCNT for write verification
  }

  optional<bool> ok;
  if (read_length > 0) {
    ok = this->read_array(data, read_length);
  }

  return ok.value_or(false);
}

void TMC2209API::write_register(uint8_t address, int32_t value) {
  std::array<uint8_t, 8> data = {0};

  data[0] = 0x05;
  data[1] = this->driver_address_;
  data[2] = address | TMC_WRITE_BIT;
  data[3] = (value >> 24) & 0xFF;
  data[4] = (value >> 16) & 0xFF;
  data[5] = (value >> 8) & 0xFF;
  data[6] = (value) &0xFF;
  data[7] = this->crc8_(data.data(), 7);

  this->read_write_register_(&data[0], 8, 0);
  this->cache_(CACHE_WRITE, address, (uint32_t *) &value);
}

int32_t TMC2209API::read_register(uint8_t address) {
  uint32_t value;

  // Read from cache for registers with write-only access
  if (this->cache_(CACHE_READ, address, &value))
    return value;

  address = address & TMC_ADDRESS_MASK;
  std::array<uint8_t, 8> data = {0};

  data[0] = 0x05;
  data[1] = this->driver_address_;
  data[2] = address;
  data[3] = this->crc8_(data.data(), 3);

  if (!this->read_write_register_(&data[0], 4, 8))
    return 0;

  // Byte 0: Sync nibble correct?
  if (data[0] != 0x05)
    return 0;

  // Byte 1: Master address correct?
  if (data[1] != 0xFF)
    return 0;

  // Byte 2: Address correct?
  if (data[2] != address)
    return 0;

  // Byte 7: CRC correct?
  if (data[7] != this->crc8_(data.data(), 7))
    return 0;

  return encode_uint32(data[3], data[4], data[5], data[6]);
}

uint32_t TMC2209API::update_field(uint32_t data, RegisterField field, uint32_t value) {
  return (data & (~field.mask)) | ((value << field.shift) & field.mask);
}

void TMC2209API::write_field(RegisterField field, uint32_t value) {
  uint32_t reg_value = this->read_register(field.address);
  reg_value = this->update_field(reg_value, field, value);
  this->write_register(field.address, reg_value);
}

uint32_t TMC2209API::extract_field(uint32_t data, RegisterField field) {
  uint32_t value = (data & field.mask) >> field.shift;

  if (field.is_signed) {
    uint32_t base_mask = field.mask >> field.shift;
    uint32_t sign_mask = base_mask & (~base_mask >> 1);
    value = (value ^ sign_mask) - sign_mask;
  }

  return value;
}

uint32_t TMC2209API::read_field(RegisterField field) {
  uint32_t value = this->read_register(field.address);
  return this->extract_field(value, field);
}

}  // namespace tmc2209
}  // namespace esphome
