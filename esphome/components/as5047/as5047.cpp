#include "as5047.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace as5047 {

static const char *TAG = "as5047";

void AS5047Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AS5047:");
  LOG_PIN("  CS Pin: ", this->cs_);
};

void AS5047Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up hub...");
  this->spi_setup();
  while (!this->spi_is_ready()) {
  }

  int16_t read_reg_err_code;

  this->zero_pos_calibrated_ = false;
  this->initialized_ = true;

  delay_microseconds_safe(10000);

  // Flush error register
  this->read_register(AS5047P_ERRFL, AS5047P_OPT_ENABLED);

  // Read error register value after clear. Must be zero.
  read_reg_err_code = this->read_register(AS5047P_ERRFL, AS5047P_OPT_ENABLED);

  if (read_reg_err_code != 0) {
    if (read_reg_err_code > 0) {
      this->handle_error_(31);
    }
    return this->mark_failed();
  }

  this->clear_error_();

  ESP_LOGCONFIG(TAG, "Done setting up hub...");
};

void AS5047Component::clear_error_() {
  this->error_ = {0, errMesage[0]};

  // ESP_LOGE(TAG, "%d: %s" this->error_.errorCode, this->error_.msg);
}

uint8_t AS5047Component::calc_parity_(uint32_t v) {
  v ^= v >> 1;
  v ^= v >> 2;
  v = (v & 0x11111111U) * 0x11111111U;
  return (v >> 28) & 1;
}

bool AS5047Component::is_parity_ok_(uint16_t frameRx) {
  uint32_t parityReceived;
  uint32_t parityCalculated;

  //--- Calculate parity for Rx frame
  parityReceived = (frameRx & AS5047P_FRAME_PARD) >> 15;
  parityCalculated = this->calc_parity_(frameRx & (AS5047P_FRAME_DATA | AS5047P_FRAME_EF));

  return (parityCalculated == parityReceived);  // Parity check
}

int16_t AS5047Component::read_write_raw(int16_t dataTx, bool rw) {
  int16_t frameTx = (dataTx & AS5047P_FRAME_DATA) | (rw << 14) | (this->calc_parity_(dataTx) << 15);

  std::array<uint8_t, 2> data = {static_cast<uint8_t>((frameTx >> 8) & 0xFF), static_cast<uint8_t>(frameTx & 0xFF)};

  this->transfer_array(data);

  uint16_t frameRx = (static_cast<uint16_t>(data[0]) << 8) | data[1];

  //--- Check for framing error
  if (frameRx & AS5047P_FRAME_EF) {
    this->handle_error_(2);
    return -1;
  }

  //--- Parity check
  if (!this->is_parity_ok_(frameRx)) {
    this->handle_error_(3);
    return -1;
  }

  return frameRx & AS5047P_FRAME_DATA;
}

int16_t AS5047Component::read_register(uint16_t regAddr, bool devReachCheck) { return 0; }
int16_t AS5047Component::write_register(uint16_t regAddr, int16_t newRegContent, bool writeVerif, bool devReachCheck) {
  return 0;
}

int16_t AS5047Component::set_factory_settings() { return 0; }
bool AS5047Component::error_pending() { return 0; }
// Error AS5047Component::get_error() { return nullptr; }
int16_t AS5047Component::error_ack() { return 0; }
int16_t AS5047Component::set_field_in_register(uint16_t regAddr, uint16_t fieldMask, uint16_t fieldVal) { return 0; }

int16_t AS5047Component::read_position(bool extendedDiag) {
  int16_t currPos;
  int16_t diagData;

  if (this->initialized_ && this->zero_pos_calibrated_) {
    this->read_write_raw(AS5047P_ANGLECOM, AS5047P_ACCESS_READ);
    currPos = this->read_write_raw(AS5047P_DIAAGC, AS5047P_ACCESS_READ);

    if (currPos < 0) {
      return -1;
    }

    if (extendedDiag) {
      diagData = this->read_write_raw(AS5047P_NOP, AS5047P_ACCESS_READ);

      if (diagData < 0) {
        return -1;  // Framing error or parity error occured
      }

      if (diagData == 0) {
        this->handle_error_(70);  // Device not accessible on SPI line
        return -1;
      }

      if (diagData & AS5047P_DIAAGC_MAGL) {
        this->handle_error_(71);  // Magnetic field strength too low
        return -1;
      }

      if (diagData & AS5047P_DIAAGC_MAGH) {
        this->handle_error_(72);  // Magnetic field strength too high
        return -1;
      }

      if (diagData & AS5047P_DIAAGC_COF) {
        this->handle_error_(73);  // CORDIC overflow
        return -1;
      }

      if ((diagData & AS5047P_DIAAGC_LF) == 0) {
        this->handle_error_(74);  // Magnet offset compensation error
        return -1;
      }
    }

  } else if (!this->initialized_) {
    this->handle_error_(75);  // Encoder not initialized while position read attempt
    return -1;
  } else if (!this->zero_pos_calibrated_) {
    this->handle_error_(76);  // Encoder not calibrated (zero pos not set) while position read attempt
    return -1;
  }

  return currPos;
}

int16_t AS5047Component::set_zero_position() { return 0; }
int16_t AS5047Component::set_abi_resolution(uint16_t resolution) { return 0; }
int16_t AS5047Component::burn_otp(uint16_t yesImSure) { return 0; }

void AS5047Component::handle_error_(int16_t errCode) {
  //--- Allow only setting errors (no clear possible)
  if ((this->error_.errorCode == 0) && (errCode != 0)) {
    this->error_.errorCode = errCode;

    if (errCode < AS5047P_ERRMSG_COUNT) {
      this->error_.msg = errMesage[errCode];
      // ESP_LOGE(TAG, "%d: %s" this->error_.errorCode, this->error_.msg);
    } else {
      this->error_.msg = NULL;
    }
  }
}

}  // namespace as5047
}  // namespace esphome
