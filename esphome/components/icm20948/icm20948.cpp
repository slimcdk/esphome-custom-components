#include "icm20948.h"

// #include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace icm20948 {

static const char *TAG = "icm20948.sensor";

#define READ_REGISTER_CHECK(this, register, val, len) \
  if (!this->read(register, val, (size_t) len)) { \
    this->mark_failed(); \
    return {}; \
  }

uint8_t ICM20948::read_icid() {
  uint8_t val;
  READ_REGISTER_CHECK(this, ICM20X_B0_WHOAMI, &val, 8);
  return val;
};

uint8_t ICM20948::read_b0() {
  uint8_t val;
  READ_REGISTER_CHECK(this, ICM20X_B0_PWR_MGMT_1, &val, 8);
  return val;
};

void ICM20948::update() {
  uint8_t tl, th;
  uint8_t t[2];

  const bool thret = this->read(0x39, &th);
  const bool tlret = this->read(0x3A, &tl);
  const bool tret = this->read(0x39, t, 16);

  // clang-format off
  ESP_LOGD(TAG, "0x%X " BYTE_TO_BINARY_PATTERN, this->read_icid(), BYTE_TO_BINARY(this->read_b0()));

  ESP_LOGD(TAG, "%s: 0x39 = %d " BYTE_TO_BINARY_PATTERN, thret ? "OK  " : "FAIL", th, th);
  ESP_LOGD(TAG, "%s: 0x3A = %d " BYTE_TO_BINARY_PATTERN, tlret ? "OK  " : "FAIL", tl, tl);

  ESP_LOGD(TAG, "%s: 0x39[0] = %d " BYTE_TO_BINARY_PATTERN, t ? "OK  " : "FAIL", t[0], t[0]);
  ESP_LOGD(TAG, "%s: 0x39[1] = %d " BYTE_TO_BINARY_PATTERN, t ? "OK  " : "FAIL", t[1], t[1]);
  // clang-format on
}

}  // namespace icm20948
}  // namespace esphome
