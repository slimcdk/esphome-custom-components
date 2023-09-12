#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/tmc2209/tmc2209_api.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace tmc {

class TMC2209Sensor : public PollingComponent, public sensor::Sensor {
 public:
  TMC2209Sensor(TMC2209 *parent) : parent_(parent){};

  float get_setup_priority() const override { return setup_priority::DATA; }
  void dump_config() override;
  void setup() override;
  void update() override;

  void set_stallguard_result_sensor(sensor::Sensor *sg_result_sensor_) { sg_result_sensor_ = sg_result_sensor_; }

 protected:
  TMC2209 *parent_;

  sensor::Sensor *sg_result_sensor_{nullptr};
};

}  // namespace tmc
}  // namespace esphome
