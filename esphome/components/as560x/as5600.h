#pragma once

#include "esphome/core/component.h"
#include "as560x.h"

namespace esphome {
namespace as560x {

#define AS5600_REGISTER_MPOS 0x03
#define AS5600_REGISTER_MANG 0x05

class AS5600 : public Component, public AS560X {
 public:
  AS5600() = default;
  void dump_config() override;
  void setup() override;
};

template<typename... Ts> class AS5600ConfigAction : public Action<Ts...>, public Parented<AS5600> {
 public:
  // explicit AS5600ConfigAction(AS5600 *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint16_t, zero_position)
  TEMPLATABLE_VALUE(uint16_t, stop_position)
  TEMPLATABLE_VALUE(uint16_t, maximum_angle)

  void play(Ts... x) override {
    if (this->zero_position_.has_value())
      this->parent_->set_zero_position(this->zero_position_.value(x...));

    if (this->stop_position.has_value())
      this->parent_->set_stop_position(this->stop_position.value(x...));

    if (this->maximum_angle_.has_value())
      this->parent_->set_maximum_angle(this->maximum_angle_.value(x...));
  }
};

}  // namespace as560x
}  // namespace esphome
