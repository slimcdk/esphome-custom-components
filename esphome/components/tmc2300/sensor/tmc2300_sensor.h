#pragma once
#include "esphome/components/tmc2300/tmc2300_api_registers.h"
#include "esphome/components/tmc2300/tmc2300_component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2300_sensor {

using namespace tmc2300;

class StallGuardResultSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

class MotorLoadSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

class ActualCurrentSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

class PWMScaleSumSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

class PWMScaleAutoSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

class PWMOFSAutoSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

class PWMGradAutoSensor : public PollingComponent, public sensor::Sensor, public Parented<TMC2300Component> {
  void dump_config();
  void update() override;
};

}  // namespace tmc2300_sensor
}  // namespace esphome
