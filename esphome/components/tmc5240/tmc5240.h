#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/stepper/stepper.h"

#if defined(TMC5240_USE_SPI)
#include "esphome/components/spi/spi.h"
#endif
#if defined(TMC5240_USE_UART)
#include "esphome/components/uart/uart.h"
#endif

extern "C" {
#include <ic/TMC5240/TMC5240.h>
#include <helpers/Bits.h>
#include <helpers/Macros.h>
}

namespace esphome {
namespace tmc5240 {

#define LOG_TMC5240(this) \
  LOG_PIN("  ENN Pin: ", this->enn_pin_); \
  LOG_PIN("  DIAG0 Pin: ", this->diag0_pin_); \
  LOG_PIN("  DIAG1 Pin: ", this->diag1_pin_); \
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X (revision: 0x%02X)", this->get_version(), this->get_revision()); \
  ESP_LOGCONFIG(TAG, "  External oscillator: %s", this->get_using_external_oscillator() ? "True" : "False"); \
  ESP_LOGCONFIG(TAG, "  Encoder latched: %d", this->enable_encoder_position()); \
  if (this->get_adc_err()) { \
    ESP_LOGE(TAG, "  Faulty ADC detected! Advised not to use ADC type input!"); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Supply Voltage: %.0f mV", this->get_vsupply()); \
  }

enum DriverEvent {
  DIAG0_TRIGGERED,
  DIAG1_TRIGGERED,
  STALLED,
};

class TMC5240Stepper;  // forwared declare

static TMC5240Stepper *components[TMC5240_NUM_COMPONENTS];
static uint16_t component_index = 0;

class TMC5240Stepper : public Component, public stepper::Stepper {
 public:
  TMC5240Stepper();

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_enn_pin(GPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag0_pin(GPIOPin *pin) { this->diag0_pin_ = pin; }
  void set_diag1_pin(GPIOPin *pin) { this->diag1_pin_ = pin; }

  // TMC - API wrappers
  virtual TMC5240BusType get_bus_type() const = 0;
  void write_register(uint8_t address, int32_t value) { tmc5240_writeRegister(this->id_, address, value); }
  void write_field(RegisterField field, uint32_t value) { tmc5240_fieldWrite(this->id_, field, value); }
  int32_t read_register(uint8_t address) { return tmc5240_readRegister(this->id_, address); }
  uint32_t read_field(RegisterField field) { return tmc5240_fieldRead(this->id_, field); }

  void set_xactual(int32_t value);
  void set_vmax(int32_t max);
  void set_amax(int32_t max);
  void set_dmax(int32_t max);
  void set_tvmax(int32_t max);
  void set_enc_const(float value);
  void set_shaft_direction(bool invert);
  void enable_encoder_position(bool enable);
  void chopconf_mres(uint8_t index);
  void enc_deviation(uint32_t deviation);
  void set_microsteps(uint16_t ms);
  bool get_using_external_oscillator();
  bool get_adc_err();
  bool get_shaft_direction();
  bool enable_encoder_position();
  uint8_t get_encmode_ignore_ab();
  uint8_t get_version();
  uint8_t get_revision();
  uint8_t chopconf_mres();
  uint16_t get_microsteps();
  uint32_t enc_deviation();
  int32_t get_xactual();
  int32_t get_vactual();
  int32_t get_sg4_result();
  int32_t get_x_enc();
  float get_vsupply();
  float get_temp();
  float get_enc_const();

  void add_on_alert_callback(std::function<void(DriverEvent)> &&callback) {
    this->on_alert_callback_.add(std::move(callback));
  }

 protected:
  GPIOPin *enn_pin_;
  GPIOPin *diag0_pin_;
  GPIOPin *diag1_pin_;

  uint16_t id_;  // used for tmcapi id index and esphome global component index

  CallbackManager<void(DriverEvent)> on_alert_callback_{};
};

/** SPI */
#if defined(TMC5240_USE_SPI)
class TMC5240SPIStepper : public TMC5240Stepper,
                          public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH,
                                                spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_10MHZ> {
 public:
  TMC5240SPIStepper() = default;
  void setup() override {
    this->spi_setup();
    TMC5240Stepper::setup();
  };

  TMC5240BusType get_bus_type() const override { return IC_BUS_SPI; }
};
#endif
/** End of SPI */

/** UART */
#if defined(TMC5240_USE_UART)
class TMC5240UARTStepper : public TMC5240Stepper, public uart::UARTDevice {
 public:
  TMC5240UARTStepper() = default;

  void set_uart_address(uint8_t address) { this->address_ = address; }
  uint8_t get_uart_address() { return this->address_; }

  TMC5240BusType get_bus_type() const override { return IC_BUS_UART; }

 protected:
  uint8_t address_{0x00};
};
#endif
/** End of UART */

}  // namespace tmc5240
}  // namespace esphome
