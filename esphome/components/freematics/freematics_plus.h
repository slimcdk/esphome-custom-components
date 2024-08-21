#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

// #include <OBD2UART.h>
// #include <FreematicsPlus.h>

namespace esphome {
namespace freematics {

class FreematicsPlus : public sensor::Sensor, public PollingComponent, public uart::UARTDevice {
 public:
  FreematicsPlus() = default;

  /* Method overrides */
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;
  void setup() override;
  void update() override;

 protected:
  // COBD obd;
};

}  // namespace freematics
}  // namespace esphome
