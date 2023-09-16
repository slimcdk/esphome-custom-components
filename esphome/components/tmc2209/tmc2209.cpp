
#include "esphome/core/log.h"

#include "esphome/components/pulse_counter/pulse_counter_sensor.h"

#include "tmc2209.h"

namespace esphome {
namespace tmc {

static const char *TAG = "tmc2209.stepper";

extern "C" {
/**
 * TMC-API hardware wrappers
 **/

void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209Stepper *comp = components[channel];
  comp->write_array(data, writeLength);

  // chop off transmitted bytes from the buffer and flush due to one-wire uart filling up rx when transmitting
  comp->read_array(data, writeLength);
  comp->flush();

  if (readLength) {
    comp->read_array(data, readLength);
  }
}

uint8_t tmc2209_CRC8(uint8_t *data, size_t length) { return tmc_CRC8(data, length, 0); }

void tmc2209_state_callback(TMC2209TypeDef *driver_, ConfigState state) {
  TMC2209Stepper *comp = components[driver_->config->channel];
  comp->set_config_state(state);
};
}

void IRAM_ATTR HOT IndexStore::pin_intr(IndexStore *arg) {
  if ((arg->target_ - arg->current_) > 0) {
    arg->current_ = arg->current_ + 1;
  } else {
    arg->current_ = arg->current_ - 1;
  }

  if (arg->current_ == arg->target_) {
    arg->target_reached_ = true;
  }
};

void IRAM_ATTR HOT TMC2209Stepper::diag_gpio_intr(TMC2209Stepper *driver_) { driver_->fault_detected_ = true; };

// Global list of components for TMC-API to access the UART
TMC2209Stepper::TMC2209Stepper() {
  this->channel_ = ++tmc2209_stepper_global_channel_index;
  components[this->channel_] = this;
}

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");
  LOG_PIN("  Enn Pin: ", this->enn_pin_);
  LOG_PIN("  Diag Pin: ", this->diag_pin_);
  LOG_PIN("  Index Pin: ", this->index_pin_);

  ESP_LOGCONFIG(TAG, "  Detected Version: 0x%02X", this->ioin_chip_version());
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps());
  ESP_LOGCONFIG(TAG, "  Dedge: %d", this->chopconf_dedge());
  LOG_STEPPER(this);
}

void TMC2209Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  // Driver enable pin
  this->enn_pin_->setup();
  this->enn_pin_->digital_write(true);
  this->enn_pin_state_ = true;

  // Interupt handling for index/stepping events
  this->index_pin_->setup();
  this->index_pin_->attach_interrupt(IndexStore::pin_intr, &this->index_store_, gpio::INTERRUPT_RISING_EDGE);

  // Interupt handling for diag/driver error events
  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(TMC2209Stepper::diag_gpio_intr, this, gpio::INTERRUPT_RISING_EDGE);

  // TMC-API
  tmc_fillCRC8Table((uint8_t) 0b100000111, true, 0);
  tmc2209_init(&this->driver_, this->channel_, this->address_, &this->config_, &tmc2209_defaultRegisterResetState[0]);
  tmc2209_setCallback(&this->driver_, tmc2209_state_callback);  // TODO: maybe remove entirely

  ESP_LOGD(TAG, "TMC2209 Reset %d", tmc2209_reset(&this->driver_));
  while (this->driver_.config->state != CONFIG_READY) {
    tmc2209_periodicJob(&this->driver_, 0);
  }

  this->gconf_pdn_disable(true);       // Prioritize UART communication by disabling configuration pin.
  this->gconf_mstep_reg_select(true);  // Use MSTEP register to set microstep resolution
  this->gconf_index_step(true);        // Configure stepping pulses on index pin
  this->blank_time(16);
  // this->chopconf_dedge(false);

  const auto ic_version = this->ioin_chip_version();
  if (ic_version != TMC2209_DEFAULT_CHIP_VERSION) {
    ESP_LOGW(TAG, "Non-default TMC2209 version detected %X. Expected %X", ic_version, TMC2209_DEFAULT_CHIP_VERSION);
  }

  this->set_interval(1, [this] { tmc2209_periodicJob(&this->driver_, 0); });
  this->set_interval(1, [this] { this->check_position_(); });

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::check_position_() {
  if (this->index_store_.target_reached_) {
    this->stop();
  }
  this->current_position = this->index_store_.current_;
}

void TMC2209Stepper::loop() {
  if (this->fault_detected_) {
    this->fault_detected_ = false;
    ESP_LOGI(TAG, "Fault detected! %d %d %d", this->gstat_reset(), this->gstat_drv_err(), this->gstat_uv_cp());
    this->on_fault_signal_callback_.call();
  }
}

void TMC2209Stepper::stop() {
  this->disable();
  this->vactual(0);
}

void TMC2209Stepper::set_target(int32_t steps) {
  if (this->current_position == steps)
    return;

  this->target_position = steps;
  this->index_store_.target_ = this->target_position;
  const int32_t relative_position = (this->target_position - this->current_position);
  this->index_store_.target_reached_ = false;
  const int32_t velocity = this->max_speed_ * (relative_position < 0 ? 1 : -1);
  this->enable();
  this->vactual(velocity);
}

void TMC2209Stepper::report_position(int32_t steps) {
  this->index_store_.current_ = steps;
  this->current_position = steps;
}

void TMC2209Stepper::set_config_state(ConfigState state) {
  this->cfg_state_ = state;
  ESP_LOGVV(TAG, "Received new config state from TMC-API -> %d", state);
}

/***
 * Human friendly set-/getters
 *
 ***/

void TMC2209Stepper::enable(bool enable) {
  this->enn_pin_->digital_write(enable);
  this->enn_pin_state_ = enable;
}

uint16_t TMC2209Stepper::get_microsteps() {
  const uint8_t index = this->chopconf_mres();
  switch (index) {
    case 0:
      return 256;
    case 1:
      return 128;
    case 2:
      return 64;
    case 3:
      return 32;
    case 4:
      return 16;
    case 5:
      return 8;
    case 6:
      return 4;
    case 7:
      return 2;
    case 8:
      return 1;
    default:
      return 0;
  }
}

void TMC2209Stepper::set_microsteps(uint16_t ms) {
  switch (ms) {
    case 1:
      return this->chopconf_mres(8);
    case 2:
      return this->chopconf_mres(7);
    case 4:
      return this->chopconf_mres(6);
    case 8:
      return this->chopconf_mres(5);
    case 16:
      return this->chopconf_mres(4);
    case 32:
      return this->chopconf_mres(3);
    case 64:
      return this->chopconf_mres(2);
    case 128:
      return this->chopconf_mres(1);
    case 256:
      return this->chopconf_mres(0);
    default:
      return this->chopconf_mres(0);
  }
}

float TMC2209Stepper::stallguard_load() {
  const uint16_t result = this->stallguard_sgresult();
  return (510.0 - (float) result) / (510.0 - (float) this->stallguard_sgthrs_ * 2.0);
}

void TMC2209Stepper::velocity(int32_t velocity) {
  this->enable(velocity != 0);
  this->vactual(velocity);
}

/**
 * TMC API set-/getters
 *
 * **/

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

bool TMC2209Stepper::gconf_inverse_direction() {
  return TMC2209_FIELD_READ(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT);
}

void TMC2209Stepper::gconf_inverse_direction(bool inverse_direction) {
  TMC2209_FIELD_UPDATE(&this->driver_, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, inverse_direction);
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
