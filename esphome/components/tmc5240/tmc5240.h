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
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X (revision: 0x%02X)", \
                (uint8_t) this->read_field(TMC5240_VERSION_FIELD), \
                (uint8_t) this->read_field(TMC5240_SILICON_RV_FIELD)); \
  ESP_LOGCONFIG(TAG, "  External oscillator: %s", (bool) this->read_field(TMC5240_EXT_CLK_FIELD) ? "True" : "False"); \
  ESP_LOGCONFIG(TAG, "  Encoder latched: %d", (bool) this->read_field(TMC5240_LATCH_X_ACT_FIELD)); \
  if ((bool) this->read_field(TMC5240_ADC_ERR_FIELD)) { \
    ESP_LOGE(TAG, "  Faulty ADC detected! Advised not to use ADC type input!"); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Supply Voltage: %.0f mV", this->read_supply_voltage()); \
  }

enum DriverEvent {
  DIAG0_TRIGGERED,
  DIAG1_TRIGGERED,
  STALLED,
};

class EventHandler {
 public:
  EventHandler() = default;

  void set_callback(std::function<void()> &&callback) { this->callback_ = std::move(callback); }
  void set_callback_recover(std::function<void()> &&callback) { this->callback_recover_ = std::move(callback); }
  void set_callbacks(std::function<void()> &&callback, std::function<void()> &&callback_recover) {
    this->callback_ = std::move(callback);
    this->callback_recover_ = std::move(callback_recover);
  }

  // Check state and trigger appropriate callbacks
  void check(bool state) {
    if (state != prev_) {
      if (state) {
        if (this->callback_) {
          this->callback_();
        }
      } else {
        if (this->callback_recover_) {
          this->callback_recover_();
        }
      }
      prev_ = state;  // Update previous state only when state changes
    }
  }

 private:
  bool prev_{false};
  std::function<void()> callback_;
  std::function<void()> callback_recover_;
};

struct ISRStore {
  bool *pin_triggered_ptr{nullptr};
  static void IRAM_ATTR HOT pin_isr(ISRStore *arg) { (*(arg->pin_triggered_ptr)) = true; }
};

class TMC5240Stepper : public Component, public stepper::Stepper {
 public:
  TMC5240Stepper();

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_enn_pin(GPIOPin *pin) { this->enn_pin_ = pin; }
  void set_diag0_pin(InternalGPIOPin *pin) { this->diag0_pin_ = pin; }
  void set_diag1_pin(GPIOPin *pin) { this->diag1_pin_ = pin; }

  // TMC-API wrappers
  virtual TMC5240BusType get_bus_type() const = 0;
  void write_register(uint8_t address, int32_t value) { tmc5240_writeRegister(this->id_, address, value); }
  void write_field(RegisterField field, uint32_t value) { tmc5240_fieldWrite(this->id_, field, value); }
  int32_t read_register(uint8_t address) { return tmc5240_readRegister(this->id_, address); }
  uint32_t read_field(RegisterField field) { return tmc5240_fieldRead(this->id_, field); }

  void enable_driver(bool enable, uint32_t delay = 0);

  void set_enc_const(float value);
  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();
  uint32_t get_enc_deviation();
  int32_t get_vactual();
  float read_supply_voltage();
  float read_adc_ain();
  float read_temp();
  float get_enc_const();
  float get_motor_load();

  void add_on_alert_callback(std::function<void(DriverEvent)> &&callback) {
    this->on_alert_callback_.add(std::move(callback));
  }

 protected:
  GPIOPin *enn_pin_;
  InternalGPIOPin *diag0_pin_;
  GPIOPin *diag1_pin_;

  uint16_t id_;  // used for tmcapi id index and esphome global component index

  // Driver enable/disable helper flags
  bool driver_state_;
  bool driver_state_change_scheduled_;
  uint32_t driver_state_change_delay_;
  uint32_t current_delay_;

  EventHandler diag0_handler_{};  // Events on DIAG0
  ISRStore diag0_isr_store_{};
  bool diag0_triggered_{false};

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
  void setup() override { TMC5240Stepper::setup(); };

  void set_uart_address(uint8_t address) { this->address_ = address; }
  uint8_t get_uart_address() { return this->address_; }

  TMC5240BusType get_bus_type() const override { return IC_BUS_UART; }

 protected:
  uint8_t address_;
};
#endif
/** End of UART */

static TMC5240Stepper *components[TMC5240_NUM_COMPONENTS];
static uint16_t component_index = 0;

}  // namespace tmc5240
}  // namespace esphome
