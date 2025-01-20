#pragma once

#include "esphome/components/uart/uart.h"

namespace esphome {
namespace tmc2209_hub {

class TMC2209Hub : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  /*
    void write_array(const uint8_t *data, size_t len) { this->parent_->write_array(data, len); }
    void write_array(const std::vector<uint8_t> &data) { this->parent_->write_array(data); }
    template<size_t N> void write_array(const std::array<uint8_t, N> &data) {
      this->parent_->write_array(data.data(), data.size());
    }

    bool read_array(uint8_t *data, size_t len) { return this->parent_->read_array(data, len); }
    template<size_t N> optional<std::array<uint8_t, N>> read_array() {  // NOLINT
      std::array<uint8_t, N> res;
      if (!this->read_array(res.data(), N)) {
        return {};
      }
      return res;
    }

    int available() { return this->parent_->available(); }

    void flush() { this->parent_->flush(); }
    */
};

class TMC2209Device {
 public:
  TMC2209Device() = default;
  TMC2209Device(TMC2209Hub *parent) : parent_(parent) {}

  void set_tmc2209_hub_parent(TMC2209Hub *parent) { this->parent_ = parent; }

 protected:
  TMC2209Hub *parent_{nullptr};
};

}  // namespace tmc2209_hub
}  // namespace esphome
