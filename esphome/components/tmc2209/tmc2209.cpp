#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc2209 {

static const char *TAG = "tmc2209";

TMC2209::TMC2209(uint8_t address) : address_(address) {
  // Global list of instances to handle access in static C functions
  this->id_ = tmc2209_global_index++;
  components[this->id_] = this;
}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");
  ESP_LOGCONFIG(TAG, "  RSense: %.2f Ohm (%s)", this->rsense_, this->use_internal_rsense_ ? "Internal" : "External");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", this->ioin_chip_version());
  ESP_LOGCONFIG(TAG, "  Oscillator frequency: %d Hz", this->oscillator_freq_);
}

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  /* Configure driver for usage in ESPHome */
  this->gconf_pdn_disable(true);       // Prioritize UART communication by disabling configuration pins
  this->gconf_mstep_reg_select(true);  // Use MSTEP register to set microstep resolution
  this->gconf_internal_rsense(this->use_internal_rsense_);
  this->gconf_multistep_filt(false);
  this->gconf_iscale_analog(false);
  this->gconf_index_step(false);
  this->gconf_index_otpw(true);
  this->vactual(0);  // Stop internal generator if last shutdown was ungraceful
  /*  */

  this->configure_event_handlers();

#if defined(USE_INDEX_PIN)
  this->index_pin_->setup();
  this->index_pin_->attach_interrupt(ISRStore::pin_isr, &this->index_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->index_isr_store_.pin_triggered_ptr = &this->index_triggered_;
#endif

#if defined(USE_DIAG_PIN)
  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(ISRStore::pin_isr, &this->diag_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->diag_isr_store_.pin_triggered_ptr = &this->diag_triggered_;
#endif

  ESP_LOGCONFIG(TAG, "TMC2209 setup done.");
}

void TMC2209::handle_diag_event() {
  // TODO: Read driver error and broadcast event accordingly
  this->on_alert_callback_.call(STALLED);
}

void TMC2209::handle_index_event() {}

void TMC2209::configure_event_handlers() {
  this->diag_handler_.set_callback([this]() {
    this->on_alert_callback_.call(DIAG_TRIGGERED);
    this->handle_diag_event();
  });

  this->index_handler_.set_callback([this]() {
    this->on_alert_callback_.call(INDEX_TRIGGERED);
    this->handle_index_event();
  });

  // Temperature events
  this->nt_handler_.set_callback([this]() { this->on_alert_callback_.call(TEMPERATURE_NORMAL); });
  this->otpw_handler_.set_callback([this]() { this->on_alert_callback_.call(OVERTEMPERATURE_PREWARNING); });
  this->ot_handler_.set_callback([this]() { this->on_alert_callback_.call(OVERTEMPERATURE); });
  this->t120_handler_.set_callbacks(                                        // t120 flag
      [this]() { this->on_alert_callback_.call(TEMPERATURE_ABOVE_120C); },  // rise
      [this]() { this->on_alert_callback_.call(TEMPERATURE_BELOW_120C); }   // fall
  );
  this->t143_handler_.set_callbacks(                                        // t143 flag
      [this]() { this->on_alert_callback_.call(TEMPERATURE_ABOVE_143C); },  // rise
      [this]() { this->on_alert_callback_.call(TEMPERATURE_BELOW_143C); }   // fall
  );
  this->t150_handler_.set_callbacks(                                        // t150 flag
      [this]() { this->on_alert_callback_.call(TEMPERATURE_ABOVE_150C); },  // rise
      [this]() { this->on_alert_callback_.call(TEMPERATURE_BELOW_150C); }   // fall
  );
  this->t157_handler_.set_callbacks(                                        // t157 flag
      [this]() { this->on_alert_callback_.call(TEMPERATURE_ABOVE_157C); },  // rise
      [this]() { this->on_alert_callback_.call(TEMPERATURE_BELOW_157C); }   // fall
  );
}

void TMC2209::run_event_triggers() {
#if !defined(USE_DIAG_PIN)
  this->diag_triggered_ = this->ioin_diag();
#endif
  this->diag_handler_.check(this->diag_triggered_);
  this->diag_triggered_ = false;

#if defined(USE_INDEX_PIN)
  this->index_handler_.check(this->index_triggered_);
#endif
  this->index_triggered_ = false;

  // Temperature events
  this->t120_handler_.check(this->drv_status_t120());
  this->t143_handler_.check(this->drv_status_t143());
  this->t150_handler_.check(this->drv_status_t150());
  this->t157_handler_.check(this->drv_status_t157());
  const bool drv_status_otpw = this->drv_status_otpw();
  const bool drv_status_ot = this->drv_status_ot();
  this->nt_handler_.check(!drv_status_otpw && !drv_status_ot);  // both overtemp prewarn and overtemp must be off / gone
  this->otpw_handler_.check(drv_status_otpw);
  this->ot_handler_.check(drv_status_ot);
}

void TMC2209::loop() { this->run_event_triggers(); }

uint16_t TMC2209::get_microsteps() {
  const uint8_t mres = this->chopconf_mres();
  return 256 >> mres;
}

void TMC2209::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->chopconf_mres(mres);
}

float TMC2209::motor_load() {
  const uint16_t result = this->stallguard_sgresult();
  return (510.0 - (float) result) / (510.0 - (float) this->stallguard_sgthrs_ * 2.0);
}

float TMC2209::rms_current_hold_scale() { return this->rms_current_hold_scale_; }
void TMC2209::rms_current_hold_scale(float scale) {
  this->rms_current_hold_scale_ = scale;
  this->set_rms_current_();
}

void TMC2209::rms_current(float A) {
  this->rms_current_ = A;
  this->set_rms_current_();
}

float TMC2209::rms_current() { return this->current_scale_to_rms_current_(this->ihold_irun_irun()); }

void TMC2209::set_rms_current_() {
  uint8_t current_scale = 32.0 * 1.41421 * this->rms_current_ * (this->rsense_ + 0.02) / 0.325 - 1;

  if (current_scale < 16) {
    this->chopconf_vsense(true);
    current_scale = 32.0 * 1.41421 * this->rms_current_ * (this->rsense_ + 0.02) / 0.180 - 1;
  } else {
    this->chopconf_vsense(false);
  }

  if (current_scale > 31) {
    current_scale = 31;
    ESP_LOGW(TAG, "Selected rsense has a current limit of %.3f A", this->current_scale_to_rms_current_(current_scale));
  }

  this->ihold_irun_irun(current_scale);
  this->ihold_irun_ihold(current_scale * this->rms_current_hold_scale_);
}

float TMC2209::current_scale_to_rms_current_(uint8_t current_scaling) {
  return (current_scaling + 1) / 32.0 * (this->chopconf_vsense() ? 0.180 : 0.325) / (this->rsense_ + 0.02) / sqrtf(2);
}

void TMC2209::ihold_irun_ihold_delay_ms(uint32_t delay_in_ms) {
  this->ihold_irun_ihold_delay(((float) delay_in_ms / 262144.0) * ((float) this->oscillator_freq_ / 1000.0));
}

uint32_t TMC2209::ihold_irun_ihold_delay_ms() {
  return (this->ihold_irun_ihold_delay() * 262144) / (this->oscillator_freq_ / 1000);
}

void TMC2209::tpowerdown_ms(uint32_t delay_in_ms) {
  this->tpowerdown(((float) delay_in_ms / 262144.0) * ((float) this->oscillator_freq_ / 1000.0));
};

uint32_t TMC2209::tpowerdown_ms() { return (this->tpowerdown() * 262144) / (this->oscillator_freq_ / 1000); };

uint8_t TMC2209::get_address() { return this->address_; }

/***
 * TMC-API wrappers
 *
 **/

extern "C" {
bool tmc2209_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209 *comp = components[id];

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    comp->read_array(data, readLength);
  }
  return true;
}

uint8_t tmc2209_getNodeAddress(uint16_t id) {
  TMC2209 *comp = components[id];
  return comp->get_address();
}
}

uint16_t TMC2209::internal_step_counter() { return this->read_field(TMC2209_MSCNT_FIELD); };
int16_t TMC2209::current_a() { return this->read_field(TMC2209_CUR_A_FIELD); };
int16_t TMC2209::current_b() { return this->read_field(TMC2209_CUR_B_FIELD); };

int32_t TMC2209::vactual() { return this->read_field(TMC2209_VACTUAL_FIELD); }
void TMC2209::vactual(int32_t velocity) { this->write_field(TMC2209_VACTUAL_FIELD, velocity); }

uint32_t TMC2209::tstep() { return this->read_field(TMC2209_TSTEP_FIELD); }

void TMC2209::ihold_irun_ihold(uint8_t current) { this->write_field(TMC2209_IRUN_FIELD, current); }
uint8_t TMC2209::ihold_irun_ihold() { return this->read_field(TMC2209_IRUN_FIELD); }

void TMC2209::ihold_irun_irun(uint8_t current) { this->write_field(TMC2209_IHOLD_FIELD, current); }
uint8_t TMC2209::ihold_irun_irun() { return this->read_field(TMC2209_IHOLD_FIELD); }

uint8_t TMC2209::ihold_irun_ihold_delay() { return this->read_field(TMC2209_IHOLDDELAY_FIELD); }
void TMC2209::ihold_irun_ihold_delay(uint8_t factor) {
  if (factor > 15) {
    ESP_LOGW(TAG, "IHOLDDELAY is limited to 15. This is the raw value and not the delay in microseconds.");
    factor = 15;
  }
  this->write_field(TMC2209_IHOLDDELAY_FIELD, factor);
}
uint32_t TMC2209::otpread() { return this->read_register(TMC2209_OTP_READ); }
bool TMC2209::optread_en_spreadcycle() { return (bool) this->read_field(TMC2209_STST_FIELD); }
uint8_t TMC2209::stallguard_sgthrs() { return this->stallguard_sgthrs_; }
void TMC2209::stallguard_sgthrs(uint8_t threshold) {
  this->write_register(TMC2209_SGTHRS, (int32_t) threshold);
  this->stallguard_sgthrs_ = threshold;
}
uint16_t TMC2209::stallguard_sgresult() { return this->read_register(TMC2209_SG_RESULT); }
uint32_t TMC2209::ioin() { return this->read_register(TMC2209_IOIN); }
bool TMC2209::ioin_enn() { return this->read_field(TMC2209_ENN_FIELD); }
bool TMC2209::ioin_ms1() { return this->read_field(TMC2209_MS1_FIELD); }
bool TMC2209::ioin_ms2() { return this->read_field(TMC2209_MS2_FIELD); }
bool TMC2209::ioin_diag() { return this->read_field(TMC2209_DIAG_FIELD); }
bool TMC2209::ioin_pdn_uart() { return this->read_field(TMC2209_PDN_UART_FIELD); }
bool TMC2209::ioin_step() { return this->read_field(TMC2209_STEP_FIELD); }
bool TMC2209::ioin_spread_en() { return this->read_field(TMC2209_SEL_A_FIELD); }
bool TMC2209::ioin_dir() { return this->read_field(TMC2209_DIR_FIELD); }
int8_t TMC2209::ioin_chip_version() { return this->read_field(TMC2209_VERSION_FIELD); }
uint8_t TMC2209::transmission_counter() { return this->read_field(TMC2209_IFCNT_FIELD); }
uint16_t TMC2209::gstat() { return this->read_register(TMC2209_GSTAT); }
void TMC2209::gstat(uint16_t setting) { return this->write_register(TMC2209_GSTAT, setting); }
bool TMC2209::gstat_reset() { return this->read_field(TMC2209_RESET_FIELD); }
void TMC2209::gstat_reset(bool clear) { this->write_field(TMC2209_RESET_FIELD, clear); }
bool TMC2209::gstat_drv_err() { return this->read_field(TMC2209_DRV_ERR_FIELD); }
void TMC2209::gstat_drv_err(bool clear) { this->write_field(TMC2209_DRV_ERR_FIELD, clear); }
bool TMC2209::gstat_uv_cp() { return this->read_field(TMC2209_UV_CP_FIELD); }
void TMC2209::gstat_uv_cp(bool clear) { this->write_field(TMC2209_UV_CP_FIELD, clear); }
uint16_t TMC2209::gconf() { return this->read_register(TMC2209_GCONF); }
void TMC2209::gconf(uint16_t setting) { return this->write_register(TMC2209_GCONF, setting); }
bool TMC2209::gconf_iscale_analog() { return this->read_field(TMC2209_I_SCALE_ANALOG_FIELD); }
void TMC2209::gconf_iscale_analog(bool use_vref) { this->write_field(TMC2209_I_SCALE_ANALOG_FIELD, use_vref); }
bool TMC2209::gconf_internal_rsense() { return this->read_field(TMC2209_INTERNAL_RSENSE_FIELD); }
void TMC2209::gconf_internal_rsense(bool use_internal) {
  this->write_field(TMC2209_INTERNAL_RSENSE_FIELD, use_internal);
}
bool TMC2209::gconf_en_spreadcycle() { return this->read_field(TMC2209_EN_SPREADCYCLE_FIELD); }
void TMC2209::gconf_en_spreadcycle(bool enable) { this->write_field(TMC2209_EN_SPREADCYCLE_FIELD, enable); }
bool TMC2209::gconf_shaft() { return this->read_field(TMC2209_SHAFT_FIELD); }
void TMC2209::gconf_shaft(bool inverse) { this->write_field(TMC2209_SHAFT_FIELD, inverse); }
bool TMC2209::gconf_index_otpw() { return this->read_field(TMC2209_INDEX_OTPW_FIELD); }
void TMC2209::gconf_index_otpw(bool use_otpw) { this->write_field(TMC2209_INDEX_OTPW_FIELD, use_otpw); }
void TMC2209::gconf_index_step(bool enable) { this->write_field(TMC2209_INDEX_STEP_FIELD, enable); }
bool TMC2209::gconf_index_step() { return (bool) this->read_field(TMC2209_INDEX_STEP_FIELD); }
bool TMC2209::gconf_pdn_disable() { return this->read_field(TMC2209_PDN_DISABLE_FIELD); }
void TMC2209::gconf_pdn_disable(bool disable) { this->write_field(TMC2209_PDN_DISABLE_FIELD, disable); }
bool TMC2209::gconf_mstep_reg_select() { return this->read_field(TMC2209_MSTEP_REG_SELECT_FIELD); }
void TMC2209::gconf_mstep_reg_select(bool use) { this->write_field(TMC2209_MSTEP_REG_SELECT_FIELD, use); }
bool TMC2209::gconf_multistep_filt() { return this->read_field(TMC2209_MULTISTEP_FILT_FIELD); }
void TMC2209::gconf_multistep_filt(bool enable) { this->write_field(TMC2209_MULTISTEP_FILT_FIELD, enable); }
void TMC2209::gconf_test_mode(bool enable) { this->write_field(TMC2209_TEST_MODE_FIELD, enable); }
bool TMC2209::gconf_test_mode() { return this->read_field(TMC2209_TEST_MODE_FIELD); }
void TMC2209::fclktrim(uint8_t fclktrim) { this->write_field(TMC2209_FCLKTRIM_FIELD, fclktrim); }
uint8_t TMC2209::fclktrim() { return this->read_field(TMC2209_FCLKTRIM_FIELD); }
void TMC2209::ottrim(uint8_t ottrim) { this->write_field(TMC2209_OTTRIM_FIELD, ottrim); }
uint8_t TMC2209::ottrim() { return this->read_field(TMC2209_OTTRIM_FIELD); }
bool TMC2209::drv_status_stst() { return this->read_field(TMC2209_STST_FIELD); }
bool TMC2209::drv_status_stealth() { return this->read_field(TMC2209_STEALTH_FIELD); }
uint8_t TMC2209::drv_status_cs_actual() { return this->read_field(TMC2209_CS_ACTUAL_FIELD); }
bool TMC2209::drv_status_otpw() { return this->read_field(TMC2209_OTPW_FIELD); }
bool TMC2209::drv_status_ot() { return this->read_field(TMC2209_OT_FIELD); }
bool TMC2209::drv_status_t120() { return this->read_field(TMC2209_T120_FIELD); }
bool TMC2209::drv_status_t143() { return this->read_field(TMC2209_T143_FIELD); }
bool TMC2209::drv_status_t150() { return this->read_field(TMC2209_T150_FIELD); }
bool TMC2209::drv_status_t157() { return this->read_field(TMC2209_T157_FIELD); }
bool TMC2209::drv_status_ola() { return this->read_field(TMC2209_OLA_FIELD); }
bool TMC2209::drv_status_olb() { return this->read_field(TMC2209_OLB_FIELD); }
bool TMC2209::drv_status_s2vsa() { return this->read_field(TMC2209_S2VSA_FIELD); }
bool TMC2209::drv_status_s2vsb() { return this->read_field(TMC2209_S2VSB_FIELD); }
bool TMC2209::drv_status_s2ga() { return this->read_field(TMC2209_S2GA_FIELD); }
bool TMC2209::drv_status_s2gb() { return this->read_field(TMC2209_S2GB_FIELD); }
int32_t TMC2209::coolstep_tcoolthrs() { return this->coolstep_tcoolthrs_; }
void TMC2209::coolstep_tcoolthrs(int32_t threshold) {
  this->write_register(TMC2209_TCOOLTHRS, threshold);
  this->coolstep_tcoolthrs_ = threshold;
}
void TMC2209::blank_time(uint8_t select) { this->write_field(TMC2209_TBL_FIELD, select); }

void TMC2209::chopconf_mres(uint8_t index) { this->write_field(TMC2209_MRES_FIELD, index); }
uint8_t TMC2209::chopconf_mres() { return this->read_field(TMC2209_MRES_FIELD); }
void TMC2209::chopconf_intpol(bool enable) { this->write_field(TMC2209_INTPOL_FIELD, enable); }
bool TMC2209::chopconf_intpol() { return this->read_field(TMC2209_INTPOL_FIELD); }
bool TMC2209::chopconf_dedge() { return this->read_field(TMC2209_DEDGE_FIELD); }
void TMC2209::chopconf_dedge(bool set) { this->write_field(TMC2209_DEDGE_FIELD, set); }
bool TMC2209::chopconf_vsense() { return this->read_field(TMC2209_VSENSE_FIELD); }
void TMC2209::chopconf_vsense(bool high_sensitivity) { this->write_field(TMC2209_VSENSE_FIELD, high_sensitivity); }

void TMC2209::tpowerdown(uint8_t factor) { this->write_field(TMC2209_TPOWERDOWN_FIELD, factor); }
uint8_t TMC2209::tpowerdown() { return this->read_field(TMC2209_TPOWERDOWN_FIELD); }

}  // namespace tmc2209
}  // namespace esphome
