#pragma once

#include "tmc22xx_registers.h"

#include "esphome/core/helpers.h"
#include "esphome/components/tmc22xx_hub/tmc22xx_hub.h"

namespace esphome {
namespace tmc22xx {

static const char *TAG = "tmc22xx";

// clang-format off

#define WRITE_BIT           0x80
#define ADDRESS_MASK        0x7F

#define ACCESS_NONE         0x00
#define ACCESS_READ         0x01
#define ACCESS_WRITE        0x02
#define ACCESS_DIRTY        0x08  // Register has been written since reset -> shadow register is valid for restore

// Special Register bits
#define ACCESS_RW_SPECIAL   0x10  // Read and write are independent - different values and/or different functions
#define ACCESS_FLAGS        0x20  // Register has read or write to clear flags.
#define ACCESS_HW_PRESET    0x40  // Register has hardware presets (e.g. Factory calibrations) - do not write a default value. 0x80 is currently unused

// Permission combinations
#define ACCESS_RW           (ACCESS_READ  | ACCESS_WRITE)        // 0x03 - Read and write
#define ACCESS_RW_SEPARATE  (ACCESS_RW    | ACCESS_RW_SPECIAL)   // 0x13 - Read and write, with separate values/functions
#define ACCESS_R_FLAGS      (ACCESS_READ  | ACCESS_FLAGS)        // 0x21 - Read, has flags (read to clear)
#define ACCESS_RW_FLAGS     (ACCESS_RW    | ACCESS_FLAGS)        // 0x23 - Read and write, has flags (read or write to clear)
#define ACCESS_W_PRESET     (ACCESS_WRITE | ACCESS_HW_PRESET)    // 0x42 - Write, has hardware preset - skipped in reset routine
#define ACCESS_RW_PRESET    (ACCESS_RW    | ACCESS_HW_PRESET)    // 0x43 - Read and write, has hardware presets - skipped in reset routine

// Helper macros
#define IS_READABLE(x)      ((x) & ACCESS_READ)
#define IS_WRITABLE(x)      ((x) & ACCESS_WRITE)
#define IS_DIRTY(x)         ((x) & ACCESS_DIRTY)
#define IS_PRESET(x)        ((x) & ACCESS_HW_PRESET)
#define IS_RESETTABLE(x)    (((x) & (ACCESS_W_PRESET)) == ACCESS_WRITE) // Write bit set, Hardware preset bit not set
#define IS_RESTORABLE(x)    (((x) & ACCESS_WRITE) && (!(x & ACCESS_HW_PRESET) || (x & ACCESS_DIRTY))) // Write bit set, if it's a hardware preset register, it needs to be dirty

#define REGISTER_COUNT 128

#define TMC2202_IC_VERSION 0x20
#define TMC2208_IC_VERSION 0x20
#define TMC2209_IC_VERSION 0x21
#define TMC2224_IC_VERSION 0x20
#define TMC2225_IC_VERSION 0x20
#define TMC2226_IC_VERSION 0x21

// Helper define:
// Most register permission arrays are initialized with 128 values.
// In those fields its quite hard to have an easy overview of available
// registers. For that, ____ is defined to 0, since 4 underscores are
// very easy to distinguish from the 2-digit hexadecimal values.
// This way, the used registers (permission != ACCESS_NONE) are easily spotted
// amongst unused (permission == ACCESS_NONE) registers.
#define ____ 0x00

// Helper define:
// Default reset values are not used if the corresponding register has a
// hardware preset. Since this is not directly visible in the default
// register reset values array, N_A is used as an indicator for a preset
// value, where any value will be ignored anyways (N_A: not available).
#define N_A 0

// clang-format on

struct RegisterField {
  uint32_t mask;
  uint8_t shift;
  uint8_t address;
  bool is_signed;
};

enum CacheOperation {
  CACHE_READ,
  CACHE_WRITE,
  // Special operation: Put content into the cache without marking the entry as dirty.
  // Only used to initialize the cache with hardware defaults. This will allow reading
  // from write-only registers that have a value inside them on reset. When using this
  // operation, a restore will *not* rewrite that filled register!
  CACHE_FILL_DEFAULT,
};

static const uint8_t tmc_crc_table_poly7_reflected[256] = {
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, 0x1C, 0x8D, 0xFF,
    0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE,
    0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, 0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A,
    0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, 0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C,
    0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86,
    0x17, 0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 0x54, 0xC5,
    0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 0xE0, 0x71, 0x03, 0x92, 0xE7,
    0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
    0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35,
    0xA4, 0xD1, 0x40, 0x32, 0xA3, 0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C,
    0x2E, 0xBF, 0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 0x8C,
    0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8, 0x39, 0x4B, 0xDA,
    0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, 0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50,
    0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
};

// Register access permissions:
//   0x00: none (reserved)
//   0x01: read
//   0x02: write
//   0x03: read/write
//   0x23: read/write, flag register (write to clear)
static const uint8_t register_access_[REGISTER_COUNT] = {
    //  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
    0x03, 0x23, 0x01, 0x02, 0x02, 0x01, 0x01, 0x03, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x00 - 0x0F
    0x02, 0x02, 0x01, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x10 - 0x1F
    ____, ____, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x20 - 0x2F
    ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x30 - 0x3F
    0x02, 0x01, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x40 - 0x4F
    ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____,  // 0x50 - 0x5F
    ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, 0x01, 0x01, 0x03, ____, ____, 0x01,  // 0x60 - 0x6F
    0x03, 0x01, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____   // 0x70 - 0x7F
};

class TMC22XXAPI : public Parented<tmc22xx_hub::TMC22XXHub> {
 private:
  uint8_t dirty_bits_[REGISTER_COUNT / 8] = {0};
  int32_t shadow_register_[REGISTER_COUNT];

  void set_dirty_bit_(uint8_t index, bool value);
  bool get_dirty_bit_(uint8_t index);
  bool cache_(CacheOperation operation, uint8_t address, uint32_t *value);
  uint8_t crc8_(uint8_t *data, uint32_t bytes);

 protected:
  uint8_t address_;

 public:
  TMC22XXAPI() = default;
  TMC22XXAPI(uint8_t address) : address_(address){};

  void set_address(uint8_t address) { this->address_ = address; }

  uint8_t get_address() { return this->address_; }

  // Write or read a register (all fields) or register field (single field within register)
  void write_register(uint8_t address, int32_t value);
  int32_t read_register(uint8_t address);
  void write_field(RegisterField field, uint32_t value);
  uint32_t read_field(RegisterField field);
  uint32_t extract_field(uint32_t data, RegisterField field);
  uint32_t update_field(uint32_t data, RegisterField field, uint32_t value);
};

}  // namespace tmc22xx
}  // namespace esphome
