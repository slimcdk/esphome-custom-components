#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/spi/spi.h"

namespace esphome {
namespace as5x47 {

// Volatile Registers Addresses
#define NOP_REG 0x0000
#define ERRFL_REG 0x0001
#define PROG_REG 0x0003
#define DIAGAGC_REG 0x3FFC
#define MAG_REG 0x3FFD
#define ANGLE_REG 0x3FFE
#define ANGLECOM_REG 0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 0x0016
#define ZPOSL_REG 0x0017
#define SETTINGS1_REG 0x0018
#define SETTINGS2_REG 0x0019

#define WRITE 0
#define READ 1

// ERRFL Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t frerr : 1;
    uint16_t invcomm : 1;
    uint16_t parerr : 1;
    uint16_t unused : 13;
  } values;
} Errfl;

// PROG Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t progen : 1;
    uint16_t unused : 1;
    uint16_t otpref : 1;
    uint16_t progotp : 1;
    uint16_t unused1 : 2;
    uint16_t progver : 1;
    uint16_t unused2 : 9;
  } values;
} Prog;

// DIAAGC Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t agc : 8;
    uint16_t lf : 1;
    uint16_t cof : 1;
    uint16_t magh : 1;
    uint16_t magl : 1;
    uint16_t unused : 4;
  } values;
} Diaagc;

// MAG Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t cmag : 14;
    uint16_t unused : 2;
  } values;
} Mag;

// ANGLE Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t cordicang : 14;
    uint16_t unused : 2;
  } values;
} Angle;

// ANGLECOM Register Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t daecang : 14;
    uint16_t unused : 2;
  } values;
} Anglecom;

// ZPOSM Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__((packed)) {
    uint8_t zposm;
  } values;
} Zposm;

// ZPOSL Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__((packed)) {
    uint8_t zposl : 6;
    uint8_t compLerrorEn : 1;
    uint8_t compHerrorEn : 1;
  } values;
} Zposl;

// SETTINGS1 Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__((packed)) {
    uint8_t factory_setting : 1;
    uint8_t noiseset : 1;
    uint8_t dir : 1;
    uint8_t uvw_abi : 1;
    uint8_t daecdis : 1;
    uint8_t abibin : 1;
    uint8_t dataselect : 1;
    uint8_t pwmon : 1;
  } values;
} Settings1;

// SETTINGS2 Register Definition
typedef union {
  uint8_t raw;
  struct __attribute__((packed)) {
    uint8_t uvwpp : 3;
    uint8_t hys : 2;
    uint8_t abires : 3;
  } values;
} Settings2;

// Command Frame  Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t command_frame : 14;
    uint16_t rw : 1;
    uint16_t parc : 1;
  } values;
} CommandFrame;

// ReadData Frame  Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t data : 14;
    uint16_t ef : 1;
    uint16_t pard : 1;
  } values;
} ReadDataFrame;

// WriteData Frame  Definition
typedef union {
  uint16_t raw;
  struct __attribute__((packed)) {
    uint16_t data : 14;
    uint16_t low : 1;
    uint16_t pard : 1;
  } values;
} WriteDataFrame;

class AS5X47Component : public Component,
                        public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_200KHZ> {
 public:
  AS5X47Component() = default;
  void dump_config() override;
  void setup() override;
  void loop() override;

  float read_angle();

 protected:
  void write_data_(uint16_t command, uint16_t value);
  uint16_t read_data_(uint16_t command, uint16_t nop_command);

  ReadDataFrame read_register_(uint16_t register_address);
  void write_register_(uint16_t register_address, uint16_t register_value);

  uint16_t transfer16(uint16_t data);

  bool is_even_(uint16_t data);
};

}  // namespace as5x47
}  // namespace esphome
