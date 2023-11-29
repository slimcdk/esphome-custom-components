#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/stepper/stepper.h"

extern "C" {
#include <ic/TMC5240/TMC5240.h>
}

namespace esphome {
namespace tmc5240 {

#define LOG_TMC5240(this) \
  LOG_PIN("  Enn Pin: ", this->enn_pin_); \
  LOG_PIN("  DIAG0 Pin: ", this->enn_pin_); \
  LOG_PIN("  DIAG1 Pin: ", this->enn_pin_); \
  ESP_LOGCONFIG( \
      TAG, "  Detected IC version: 0x%02X", \
      TMC5240_FIELD_READ(&this->driver_, TMC5240_INP_OUT, TMC5240_SILICON_RV_MASK, TMC5240_SILICON_RV_SHIFT)); \
  LOG_STEPPER(this);

class TMC5240;  // Forward declare
static TMC5240 *components[TMC5240_NUM_COMPONENTS];
static uint8_t tmc5240_global_component_index = 0;

class TMC5240 : public Component, public stepper::Stepper {
 public:
  TMC5240();

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;

  void set_enn_pin(GPIOPin *pin) { this->enn_pin_ = pin; }

  /*
  friend void tmc5240_writeInt(TMC5240TypeDef *tmc5240, uint8_t address, int32_t value);
  friend int32_t tmc5240_readInt(TMC5240TypeDef *tmc5240, uint8_t address);
  friend void tmc5240_writeDatagram(TMC5240TypeDef *tmc5240, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3,
  uint8_t x4);
  */

  virtual void read_write(uint8_t *buffer, size_t length) = 0;  // make tmc-api friends of TMC5240 and protect this

 protected:
  GPIOPin *enn_pin_;

  // TMC-API handlers
  uint8_t comp_index_{0};  // used for tmcapi channel index and esphome global component index
  TMC5240TypeDef driver_;
  ConfigurationTypeDef config_;
  void update_registers_();
  bool reset_();
  bool restore_();
};

}  // namespace tmc5240
}  // namespace esphome
