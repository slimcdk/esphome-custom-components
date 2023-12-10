#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/spi/spi.h"
#include "esphome/components/tmc5240/tmc5240.h"

namespace esphome {
namespace tmc5240 {

class TMC5240SPI : public tmc5240::TMC5240,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_TRAILING,
                                         spi::DATA_RATE_10MHZ> {
 public:
  void setup() override;
  void dump_config() override;

  bool get_spi_status_status_stop_r_() { return (this->spi_status_ >> 7) & 1; };
  bool get_spi_status_status_stop_l_() { return (this->spi_status_ >> 6) & 1; };
  bool get_spi_status_position_reached_() { return (this->spi_status_ >> 5) & 1; };
  bool get_spi_status_velocity_reached_() { return (this->spi_status_ >> 4) & 1; };
  bool get_spi_status_standstill_() { return (this->spi_status_ >> 3) & 1; };
  bool get_spi_status_sg2_() { return (this->spi_status_ >> 2) & 1; };
  bool get_spi_status_driver_error_() { return (this->spi_status_ >> 1) & 1; };
  bool get_spi_status_reset_flag_() { return (this->spi_status_ >> 0) & 1; };

  void read_write(uint8_t *buffer, size_t length) override;  // TODO: protect

 protected:
  uint8_t spi_status_;
  void set_spi_status(uint8_t ss);
};

}  // namespace tmc5240
}  // namespace esphome
