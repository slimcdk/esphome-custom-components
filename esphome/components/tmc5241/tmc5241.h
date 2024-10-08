// file: esphome/components/tmc5241/tmc5241.h
/**
 * Communication agnostic TMC5241 base component.
 * Holds a global component index and definitions to write and read registers.
 */

#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "tmc5241_registers.h"

#define CAST_Sn_TO_S32(value, n) ((value) | (((value) & ((uint32_t) 1 << ((n) -1))) ? ~(((uint32_t) 1 << (n)) - 1) : 0))

namespace esphome {
namespace tmc5241 {

static const char *TAG = "tmc5240.stepper";

class TMC5241 : public Component {
 public:
  TMC5241() = default;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag0_pin(InternalGPIOPin *pin) { this->diag0_pin_ = pin; }
  void set_diag1_pin(InternalGPIOPin *pin) { this->diag1_pin_ = pin; }

  float read_supply_voltage_mV();
  float read_adc_ain_mV();
  float read_temp_C();
  int32_t get_vactual();

  virtual void write_register(uint8_t address, int32_t value) = 0;
  virtual int32_t read_register(uint8_t address) = 0;

  uint32_t read_field(RegisterField field);
  void write_field(RegisterField field, uint32_t value);
  uint32_t extract_field(uint32_t data, RegisterField field);

 protected:
  InternalGPIOPin *enn_pin_;
  InternalGPIOPin *diag0_pin_;
  InternalGPIOPin *diag1_pin_;
};

}  // namespace tmc5241
}  // namespace esphome
