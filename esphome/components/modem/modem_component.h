#pragma once

#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

#include <string.h>

namespace esphome {
namespace modem {

class ModemComponent : public Component, public uart::UARTDevice {
 public:
  ModemComponent() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const { return setup_priority::WIFI; }  // TODO: Add PPP in core
};

}  // namespace modem
}  // namespace esphome
