#pragma once

#include "esphome/core/component.h"
#include "as560x.h"

namespace esphome {
namespace as560x {

#define AS5601_REGISTER_ABN 0x09
#define AS5601_REGISTER_PUSHTHR 0x0A

class AS5601 : public Component, public AS560X {
 public:
  AS5601() = default;
  void dump_config() override;
  void loop() override;

  void set_ab_resolution(uint16_t value);
  void set_push_threshold(uint16_t value);

 protected:
  uint16_t ab_resolution_{8};
  uint16_t push_threshold_{0};
};

template<typename... Ts> class AS5601SetAction : public Action<Ts...>, public Parented<AS5601> {
 public:
  TEMPLATABLE_VALUE(uint16_t, zero_position)
  TEMPLATABLE_VALUE(uint16_t, ab_resolution)
  TEMPLATABLE_VALUE(uint16_t, push_threshold)

  void play(Ts... x) override {
    if (this->zero_position_.has_value())
      this->parent_->set_zero_position(this->zero_position_.value(x...));

    if (this->ab_resolution_.has_value()) {
      // ESP_LOGI("AS5601", "Received new AB value %d", this->ab_resolution_.value(x...));
      this->parent_->set_ab_resolution(this->ab_resolution_.value(x...));
    }

    if (this->push_threshold_.has_value())
      this->parent_->set_push_threshold(this->push_threshold_.value(x...));
  }
};

}  // namespace as560x
}  // namespace esphome
