#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/tmc2300/tmc2300.h"

namespace esphome {
namespace tmc2300 {

class StallGuardResultSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300> {
  void dump_config();
  void update() override;
};

class MotorLoadSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300> {
  void dump_config();
  void update() override;
};

class ActualCurrentSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300> {
  void dump_config();
  void update() override;
};

}  // namespace tmc2300
}  // namespace esphome
