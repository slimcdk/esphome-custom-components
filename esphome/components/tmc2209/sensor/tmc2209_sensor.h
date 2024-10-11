#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/tmc2209/tmc2209.h"

namespace esphome {
namespace tmc2209 {

class StallGuardResultSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2209> {
  void dump_config();
  void update() override;
};

class MotorLoadSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2209> {
  void dump_config();
  void update() override;
};

class ActualCurrentSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2209> {
  void dump_config();
  void update() override;
};

}  // namespace tmc2209
}  // namespace esphome
