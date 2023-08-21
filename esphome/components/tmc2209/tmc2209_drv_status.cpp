#include "esphome/core/log.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

uint32_t TMC2209::read_driver_status() { return tmc2209_readInt(&this->driver_, TMC2209_DRVSTATUS); }

void TMC2209::update_driver_status() {
  this->drv_status_ = this->read_driver_status();
  this->drv_status_last_read_ = (time_t) millis();
}

bool TMC2209::read_drv_status_stst() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_STST_MASK, TMC2209_STST_SHIFT);
}

bool TMC2209::read_drv_status_stealth() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_STEALTH_MASK, TMC2209_STEALTH_SHIFT);
}

uint8_t TMC2209::read_drv_status_cs_actual() {
  return (uint8_t) (((this->drv_status_) & (TMC2209_CS_ACTUAL_MASK)) >> (TMC2209_CS_ACTUAL_SHIFT));
}

bool TMC2209::read_drv_status_otpw() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OTPW_MASK, TMC2209_OTPW_SHIFT);
}

bool TMC2209::read_drv_status_ot() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OT_MASK, TMC2209_OT_SHIFT);
}

bool TMC2209::read_drv_status_t120() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T120_MASK, TMC2209_T120_SHIFT);
}

bool TMC2209::read_drv_status_t143() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T143_MASK, TMC2209_T143_SHIFT);
}

bool TMC2209::read_drv_status_t150() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T150_MASK, TMC2209_T150_SHIFT);
}

bool TMC2209::read_drv_status_t157() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T157_MASK, TMC2209_T157_SHIFT);
}

bool TMC2209::read_drv_status_ola() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OLA_MASK, TMC2209_OLA_SHIFT);
}

bool TMC2209::read_drv_status_olb() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OLB_MASK, TMC2209_OLB_SHIFT);
}

bool TMC2209::read_drv_status_s2vsa() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2VSA_MASK, TMC2209_S2VSA_SHIFT);
}

bool TMC2209::read_drv_status_s2vsb() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2VSB_MASK, TMC2209_S2VSB_SHIFT);
}

bool TMC2209::read_drv_status_s2ga() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2GA_MASK, TMC2209_S2GA_SHIFT);
}

bool TMC2209::read_drv_status_s2gb() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2GB_MASK, TMC2209_S2GB_SHIFT);
}

bool TMC2209::get_drv_status_stst() {
  return (bool) (((this->drv_status_) & (TMC2209_STST_MASK)) >> (TMC2209_STST_SHIFT));
}

bool TMC2209::get_drv_status_stealth() {
  return (bool) (((this->drv_status_) & (TMC2209_STEALTH_MASK)) >> (TMC2209_STEALTH_SHIFT));
}

uint8_t TMC2209::get_drv_status_cs_actual() {
  return (uint8_t) (((this->drv_status_) & (TMC2209_CS_ACTUAL_MASK)) >> (TMC2209_CS_ACTUAL_SHIFT));
}

bool TMC2209::get_drv_status_otpw() {
  return (bool) (((this->drv_status_) & (TMC2209_OTPW_MASK)) >> (TMC2209_OTPW_SHIFT));
}

bool TMC2209::get_drv_status_ot() { return (bool) (((this->drv_status_) & (TMC2209_OT_MASK)) >> (TMC2209_OT_SHIFT)); }

bool TMC2209::get_drv_status_t120() {
  return (bool) (((this->drv_status_) & (TMC2209_T120_MASK)) >> (TMC2209_T120_SHIFT));
}

bool TMC2209::get_drv_status_t143() {
  return (bool) (((this->drv_status_) & (TMC2209_T143_MASK)) >> (TMC2209_T143_SHIFT));
}

bool TMC2209::get_drv_status_t150() {
  return (bool) (((this->drv_status_) & (TMC2209_T150_MASK)) >> (TMC2209_T150_SHIFT));
}

bool TMC2209::get_drv_status_t157() {
  return (bool) (((this->drv_status_) & (TMC2209_T157_MASK)) >> (TMC2209_T157_SHIFT));
}

bool TMC2209::get_drv_status_ola() {
  return (bool) (((this->drv_status_) & (TMC2209_OLA_MASK)) >> (TMC2209_OLA_SHIFT));
}

bool TMC2209::get_drv_status_olb() {
  return (bool) (((this->drv_status_) & (TMC2209_OLB_MASK)) >> (TMC2209_OLB_SHIFT));
}

bool TMC2209::get_drv_status_s2vsa() {
  return (bool) (((this->drv_status_) & (TMC2209_S2VSA_MASK)) >> (TMC2209_S2VSA_SHIFT));
}

bool TMC2209::get_drv_status_s2vsb() {
  return (bool) (((this->drv_status_) & (TMC2209_S2VSB_MASK)) >> (TMC2209_S2VSB_SHIFT));
}

bool TMC2209::get_drv_status_s2ga() {
  return (bool) (((this->drv_status_) & (TMC2209_S2GA_MASK)) >> (TMC2209_S2GA_SHIFT));
}

bool TMC2209::get_drv_status_s2gb() {
  return (bool) (((this->drv_status_) & (TMC2209_S2GB_MASK)) >> (TMC2209_S2GB_SHIFT));
}

}  // namespace tmc
}  // namespace esphome
