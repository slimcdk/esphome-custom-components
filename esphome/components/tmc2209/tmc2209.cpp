#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

#if defined(USE_INDEX_PIN)
void IRAM_ATTR HOT TMC2209ISRStore::pulse_isr(TMC2209ISRStore *arg) {
  (*(arg->current_position_ptr)) += *(arg->direction_ptr);
  if (*(arg->current_position_ptr) == *(arg->target_position_ptr)) {
    arg->stop();
  }
}
#endif

#if defined(USE_DIAG_PIN)
void IRAM_ATTR HOT TMC2209ISRStore::fault_isr(TMC2209ISRStore *arg) {
  (*(arg->stall_detected_ptr)) = true;
#if defined(FAULT_ISR_DISABLE_DRIVER)
  arg->stop();
#endif
}
#endif

void IRAM_ATTR HOT TMC2209ISRStore::stop() {
  this->enn_pin_.digital_write(false);
  *(this->driver_is_enabled_ptr) = false;
  *(this->direction_ptr) = Direction::NONE;  // TODO: maybe remove
  *(this->target_position_ptr) = *(this->current_position_ptr);
}

TMC2209Stepper::TMC2209Stepper(bool use_internal_rsense) : use_internal_rsense_(use_internal_rsense) {
  // Global list of driver to work around TMC channel index
  this->index_ = tmc2209_stepper_global_index++;
  components[this->index_] = this;

  // Initialize TMC-API object
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2209_init(&this->driver_, this->index_, this->address_, &this->config_, &tmc2209_defaultRegisterResetState[0]);
}

void TMC2209Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  this->enn_pin_->setup();
#if defined(USE_INDEX_PIN)
  this->index_pin_->setup();
#endif
#if defined(USE_DIAG_PIN)
  this->diag_pin_->setup();
#endif

  // Fill default configuration for driver (TODO: retry call instead)
  if (!tmc2209_reset(&this->driver_)) {
    this->mark_failed();
    return;
  }

  // Write initial configuration
  for (uint8_t i = 0; i < 255 && this->driver_.config->state != CONFIG_READY; i++) {
    this->update_registers_();
  }

  if (this->driver_.config->state != CONFIG_READY) {
    ESP_LOGE(TAG, "Failed to communicate with the driver");
    this->mark_failed();
    return;
  }

  // Configure the driver for UART control
  this->gconf_pdn_disable(true);       // Prioritize UART communication by disabling configuration pin.
  this->vactual(0);                    // Stop internal generator if last shutdown was ungraceful
  this->gconf_mstep_reg_select(true);  // Use MSTEP register to set microstep resolution
  this->gconf_internal_rsense(this->use_internal_rsense_);

#if defined(USE_DIAG_PIN) || defined(USE_INDEX_PIN)
  // Pulse tracker setup
  this->isr_store_.enn_pin_ = this->enn_pin_->to_isr();
  this->isr_store_.target_position_ptr = &this->target_position;
  this->isr_store_.current_position_ptr = &this->current_position;
  this->isr_store_.direction_ptr = &this->direction_;
  this->isr_store_.driver_is_enabled_ptr = &this->driver_is_enabled_;
  this->isr_store_.stall_detected_ptr = &this->stall_detected_;

#if defined(USE_INDEX_PIN)
  this->gconf_index_step(true);  // Configure stepping pulses on index pin
  this->index_pin_->attach_interrupt(TMC2209ISRStore::pulse_isr, &this->isr_store_, gpio::INTERRUPT_RISING_EDGE);
#endif

#if defined(USE_DIAG_PIN)
  this->diag_pin_->attach_interrupt(TMC2209ISRStore::fault_isr, &this->isr_store_, gpio::INTERRUPT_RISING_EDGE);
#endif
#endif

  // #if !defined(USE_DIAG_PIN) || !defined(USE_INDEX_PIN)
  this->set_interval(1, [this]() {
#if !defined(USE_DIAG_PIN)
    if (this->ioin_diag()) {
      this->stall_detected_ = true;
    }
#endif
#if !defined(USE_INDEX_PIN)
    this->current_position += this->direction_ * this->internal_step_counter();
#endif
  });
  // #endif

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");
  LOG_PIN("  Enn Pin: ", this->enn_pin_);

#if defined(USE_INDEX_PIN)
  LOG_PIN("  Index Pin: ", this->index_pin_);
#else
  ESP_LOGCONFIG(TAG, "  No index pin configured. Using serial for position");
#endif

#if defined(USE_DIAG_PIN)
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
#else
  ESP_LOGCONFIG(TAG, "  No diag pin configured. Using serial for fault trigger");
#endif

  ESP_LOGCONFIG(TAG, "  RSense: %.2f Ohm (%s)", this->resistance_,
                this->use_internal_rsense_ ? "Internal" : "External");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X (TMCAPI Channel: %d)", this->address_, this->index_);
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", this->ioin_chip_version());
  LOG_STEPPER(this);
}

void TMC2209Stepper::loop() {
  if (this->stall_detected_) {
    this->stall_detected_ = false;
#if defined(USE_DIAG_PIN) && defined(FAULT_ISR_DISABLE_DRIVER)
    this->stop();  // Be sure to stop the motor
#endif
    this->on_fault_signal_callback_.call();
  }

  const bool has_reached_target_ = this->has_reached_target();
  has_reached_target_ ? this->high_freq_.stop() : this->high_freq_.start();
  this->enable(!has_reached_target_);

  const int32_t to_target = (this->target_position - this->current_position);
  this->direction_ = (Direction) (to_target != 0 ? to_target / abs(to_target) : NONE);  // yield 1, -1 or 0

  this->calculate_speed_(micros());
  this->vactual(this->direction_ * (int32_t) this->current_speed_);  // TODO: write only when values change
  this->update_registers_();
}

void TMC2209Stepper::stop() {
  this->direction_ = Direction::NONE;
  this->target_position = this->current_position;
  this->vactual(0);
  this->enable(false);
}

void TMC2209Stepper::enable(bool enable) {
  // if (this->driver_is_enabled_ != enable) {
  this->driver_is_enabled_ = enable;
  this->enn_pin_->digital_write(enable);
  // }
}

uint16_t TMC2209Stepper::get_microsteps() {
  const uint8_t mres = this->chopconf_mres();
  return 256 >> mres;
}

void TMC2209Stepper::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->chopconf_mres(mres);
}

float TMC2209Stepper::motor_load() {
  const uint16_t result = this->stallguard_sgresult();
  return (510.0 - (float) result) / (510.0 - (float) this->stallguard_sgthrs_ * 2.0);
}

/***
 * TMC-API wrappers
 *
 **/

bool TMC2209Stepper::reset_() { return tmc2209_reset(&this->driver_); }
bool TMC2209Stepper::restore_() { return tmc2209_restore(&this->driver_); }
void TMC2209Stepper::update_registers_() { return tmc2209_periodicJob(&this->driver_, 0); }

extern "C" {
void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209Stepper *comp = components[channel];

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    comp->read_array(data, readLength);
  }
}

uint8_t tmc2209_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }
}

uint16_t TMC2209Stepper::internal_step_counter() { return (uint16_t) tmc2209_readInt(&this->driver_, TMC2209_MSCNT); };

int16_t TMC2209Stepper::current_a() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_MSCURACT, TMC2209_CUR_A_MASK, TMC2209_CUR_A_SHIFT);  // TODO
};

int16_t TMC2209Stepper::current_b() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_MSCURACT, TMC2209_CUR_B_MASK, TMC2209_CUR_B_SHIFT);  // TODO
};
void TMC2209Stepper::vactual(int32_t velocity) {
  TMC2209_FIELD_WRITE(&this->driver_, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, velocity);
}

void TMC2209Stepper::ihold_irun_ihold(int32_t current) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, current);
}

void TMC2209Stepper::ihold_irun_irun(int32_t current) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, current);
}

void TMC2209Stepper::ihold_irun_ihold_delay(int32_t delay) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, delay);
}
uint32_t TMC2209Stepper::otpread() { return tmc2209_readInt(&this->driver_, TMC2209_OTP_READ); }

bool TMC2209Stepper::optread_en_spreadcycle() {
  return (bool) TMC2209_FIELD_READ(&this->driver_, TMC2209_OTP_READ, TMC2209_STST_MASK, TMC2209_STST_SHIFT);
}
void TMC2209Stepper::stallguard_sgthrs(uint8_t threshold) {
  tmc2209_writeInt(&this->driver_, TMC2209_SGTHRS, (int32_t) threshold);
  this->stallguard_sgthrs_ = threshold;
}

uint8_t TMC2209Stepper::stallguard_sgthrs() { return this->stallguard_sgthrs_; }

uint16_t TMC2209Stepper::stallguard_sgresult() { return (uint16_t) tmc2209_readInt(&this->driver_, TMC2209_SG_RESULT); }

uint32_t TMC2209Stepper::ioin() { return tmc2209_readInt(&this->driver_, TMC2209_IOIN); }

bool TMC2209Stepper::ioin_enn() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_ENN_MASK, TMC2209_ENN_SHIFT);
}

bool TMC2209Stepper::ioin_ms1() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_IOIN, TMC2209_MS2_SHIFT);
}

bool TMC2209Stepper::ioin_ms2() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_MS2_MASK, TMC2209_MS2_SHIFT);
}

bool TMC2209Stepper::ioin_diag() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIAG_MASK, TMC2209_DIAG_SHIFT);
}

bool TMC2209Stepper::ioin_pdn_uart() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_PDN_UART_MASK, TMC2209_PDN_UART_SHIFT);
}

bool TMC2209Stepper::ioin_step() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_STEP_MASK, TMC2209_STEP_SHIFT);
}

bool TMC2209Stepper::ioin_spread_en() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_SEL_A_MASK, TMC2209_SEL_A_SHIFT);
}

bool TMC2209Stepper::ioin_dir() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_DIR_MASK, TMC2209_DIR_SHIFT);
}

int8_t TMC2209Stepper::ioin_chip_version() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
}

uint8_t TMC2209Stepper::transmission_counter() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_IFCNT, TMC2209_IFCNT_MASK, TMC2209_IFCNT_SHIFT);
}

uint16_t TMC2209Stepper::gstat() { return tmc2209_readInt(&this->driver_, TMC2209_GSTAT); }
void TMC2209Stepper::gstat(uint16_t setting) { return tmc2209_writeInt(&this->driver_, TMC2209_GSTAT, setting); }

bool TMC2209Stepper::gstat_reset() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT);
}

void TMC2209Stepper::gstat_reset(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_RESET_MASK, TMC2209_RESET_SHIFT, clear);
}

bool TMC2209Stepper::gstat_drv_err() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT);
}

void TMC2209Stepper::gstat_drv_err(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_DRV_ERR_MASK, TMC2209_DRV_ERR_SHIFT, clear);
}

bool TMC2209Stepper::gstat_uv_cp() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT);
}

void TMC2209Stepper::gstat_uv_cp(bool clear) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GSTAT, TMC2209_UV_CP_MASK, TMC2209_UV_CP_SHIFT, clear);
}

uint16_t TMC2209Stepper::gconf() { return tmc2209_readInt(&this->driver_, TMC2209_GCONF); }
void TMC2209Stepper::gconf(uint16_t setting) { return tmc2209_writeInt(&this->driver_, TMC2209_GCONF, setting); }

void TMC2209Stepper::gconf_iscale_analog(bool use_vref) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_I_SCALE_ANALOG_MASK, TMC2209_I_SCALE_ANALOG_SHIFT,
                       use_vref);
}

bool TMC2209Stepper::gconf_iscale_analog() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_I_SCALE_ANALOG_MASK, TMC2209_I_SCALE_ANALOG_SHIFT);
}

void TMC2209Stepper::gconf_internal_rsense(bool use_internal) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_INTERNAL_RSENSE_MASK, TMC2209_INTERNAL_RSENSE_SHIFT,
                       use_internal);
}

bool TMC2209Stepper::gconf_internal_rsense() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_INTERNAL_RSENSE_MASK, TMC2209_INTERNAL_RSENSE_SHIFT);
}

void TMC2209Stepper::gconf_en_spreadcycle(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_EN_SPREADCYCLE_MASK, TMC2209_EN_SPREADCYCLE_SHIFT,
                       enable);
}

bool TMC2209Stepper::gconf_en_spreadcycle() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_EN_SPREADCYCLE_MASK, TMC2209_EN_SPREADCYCLE_SHIFT);
}

bool TMC2209Stepper::gconf_shaft() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT);
}

void TMC2209Stepper::gconf_shaft(bool inverse) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, inverse);
}

bool TMC2209Stepper::gconf_index_otpw() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_OTPW_MASK, TMC2209_INDEX_OTPW_SHIFT);
}

void TMC2209Stepper::gconf_index_otpw(bool use_otpw) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_OTPW_MASK, TMC2209_INDEX_OTPW_SHIFT, use_otpw);
}

void TMC2209Stepper::gconf_index_step(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_STEP_MASK, TMC2209_INDEX_STEP_SHIFT, enable);
}

bool TMC2209Stepper::gconf_index_step() {
  return (bool) TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_INDEX_STEP_MASK, TMC2209_INDEX_STEP_SHIFT);
}

void TMC2209Stepper::gconf_pdn_disable(bool disable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, disable);
}

bool TMC2209Stepper::gconf_pdn_disable() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT);
}

void TMC2209Stepper::gconf_mstep_reg_select(bool use) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_MSTEP_REG_SELECT_MASK, TMC2209_MSTEP_REG_SELECT_SHIFT,
                       use);
}

bool TMC2209Stepper::gconf_mstep_reg_select() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_MSTEP_REG_SELECT_MASK,
                            TMC2209_MSTEP_REG_SELECT_SHIFT);
}

void TMC2209Stepper::gconf_multistep_filt(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_MULTISTEP_FILT_MASK, TMC2209_MULTISTEP_FILT_SHIFT,
                       enable);
}

bool TMC2209Stepper::gconf_multistep_filt() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_MULTISTEP_FILT_MASK, TMC2209_MULTISTEP_FILT_SHIFT);
}

void TMC2209Stepper::gconf_test_mode(bool enable) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_TEST_MODE_MASK, TMC2209_TEST_MODE_SHIFT, enable);
}

bool TMC2209Stepper::gconf_test_mode() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_TEST_MODE_MASK, TMC2209_TEST_MODE_SHIFT);
}

void TMC2209Stepper::fclktrim(uint8_t fclktrim) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_FCLKTRIM_MASK, TMC2209_FCLKTRIM_SHIFT, fclktrim);
}

uint8_t TMC2209Stepper::fclktrim() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_FCLKTRIM_MASK, TMC2209_FCLKTRIM_SHIFT);
}

void TMC2209Stepper::ottrim(uint8_t ottrim) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_OTTRIM_MASK, TMC2209_OTTRIM_SHIFT, ottrim);
}

uint8_t TMC2209Stepper::ottrim() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_FACTORY_CONF, TMC2209_OTTRIM_MASK, TMC2209_OTTRIM_SHIFT);
}

bool TMC2209Stepper::drv_status_stst() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_STST_MASK, TMC2209_STST_SHIFT);
}

bool TMC2209Stepper::drv_status_stealth() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_STEALTH_MASK, TMC2209_STEALTH_SHIFT);
}

uint8_t TMC2209Stepper::drv_status_cs_actual() {
  return (uint8_t) TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_CS_ACTUAL_MASK,
                                      TMC2209_CS_ACTUAL_SHIFT);
}

bool TMC2209Stepper::drv_status_otpw() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OTPW_MASK, TMC2209_OTPW_SHIFT);
}

bool TMC2209Stepper::drv_status_ot() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OT_MASK, TMC2209_OT_SHIFT);
}

bool TMC2209Stepper::drv_status_t120() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T120_MASK, TMC2209_T120_SHIFT);
}

bool TMC2209Stepper::drv_status_t143() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T143_MASK, TMC2209_T143_SHIFT);
}

bool TMC2209Stepper::drv_status_t150() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T150_MASK, TMC2209_T150_SHIFT);
}

bool TMC2209Stepper::drv_status_t157() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_T157_MASK, TMC2209_T157_SHIFT);
}

bool TMC2209Stepper::drv_status_ola() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OLA_MASK, TMC2209_OLA_SHIFT);
}

bool TMC2209Stepper::drv_status_olb() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_OLB_MASK, TMC2209_OLB_SHIFT);
}

bool TMC2209Stepper::drv_status_s2vsa() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2VSA_MASK, TMC2209_S2VSA_SHIFT);
}

bool TMC2209Stepper::drv_status_s2vsb() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2VSB_MASK, TMC2209_S2VSB_SHIFT);
}

bool TMC2209Stepper::drv_status_s2ga() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2GA_MASK, TMC2209_S2GA_SHIFT);
}

bool TMC2209Stepper::drv_status_s2gb() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_DRVSTATUS, TMC2209_S2GB_MASK, TMC2209_S2GB_SHIFT);
}

void TMC2209Stepper::coolstep_tcoolthrs(int32_t threshold) {
  tmc2209_writeInt(&this->driver_, TMC2209_TCOOLTHRS, threshold);
  this->coolstep_tcoolthrs_ = threshold;
}

int32_t TMC2209Stepper::coolstep_tcoolthrs() { return this->coolstep_tcoolthrs_; }

void TMC2209Stepper::blank_time(uint8_t select) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, select);
}

void TMC2209Stepper::chopconf_mres(uint8_t index) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, index);
}

uint8_t TMC2209Stepper::chopconf_mres() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT);
}

bool TMC2209Stepper::chopconf_dedge() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_CHOPCONF, TMC2209_DEDGE_MASK, TMC2209_DEDGE_SHIFT);
}

void TMC2209Stepper::chopconf_dedge(bool set) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_CHOPCONF, TMC2209_DEDGE_MASK, TMC2209_DEDGE_SHIFT, set);
}

}  // namespace tmc
}  // namespace esphome
