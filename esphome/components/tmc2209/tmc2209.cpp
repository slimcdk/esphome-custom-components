#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "tmc2209.h"

namespace esphome {
namespace tmc2209 {

static const char *TAG = "tmc2209";

TMC2209::TMC2209(uint8_t address, uint32_t clock_frequency) : address_(address), clock_frequency_(clock_frequency) {
  // Global list of instances to handle access in static C functions
  this->id_ = component_index++;
  components[this->id_] = this;
}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209:");

  LOG_PIN("  DIAG Pin: ", this->diag_pin_);
  ESP_LOGCONFIG(TAG, "  RSense: %.2f Ohm (%s)", this->rsense_, this->use_internal_rsense_ ? "Internal" : "External");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz", this->clock_frequency_);

  const int8_t icv_ = this->read_ioin_chip_version();
  if (std::isnan(icv_) || icv_ == 0) {
    ESP_LOGE(TAG, "  Unable to read IC version. Is the driver powered and wired correctly?");
  } else if (icv_ == TMC2209_IC_VERSION_33) {
    ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", icv_);
  } else {
    ESP_LOGE(TAG, "  Detected unknown IC version: 0x%02X", icv_);
  }
  if (this->is_failed()) {
    return;
  }

  uint8_t otpw, ot;
  std::tie(otpw, ot) = this->unpack_ottrim_values(this->read_ottrim());
  // auto [otpw, ot] = this->unpack_ottrim_values(this->read_ottrim()); // c++17 required
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot);
}

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209...");

  while (!this->is_ready()) {
  };

  const int8_t icv_ = this->read_ioin_chip_version();
  if (icv_ != TMC2209_IC_VERSION_33) {
    this->status_set_error("Failed to communicate with driver");
    this->mark_failed();
  }

  /* Configure driver for basic usage in ESPHome. This is the GCONF register */
  /* bit: 0 = 0  */ this->write_gconf_iscale_analog(false);
  /* bit: 1 = 0|1*/ this->write_gconf_internal_rsense(this->use_internal_rsense_);
  /* bit: 2 = 0  */ this->write_gconf_en_spreadcycle(false);
  /* bit: 3 = 0  */ this->write_gconf_shaft(false);
  /* bit: 4 = 1  */ this->write_gconf_index_otpw(true);
  /* bit: 5 = 0  */ this->write_gconf_index_step(false);
  /* bit: 6 = 1  */ this->write_gconf_pdn_disable(
      true);  // Prioritize UART communication by disabling configuration pins
  /* bit: 7 = 1  */ this->write_gconf_mstep_reg_select(true);  // Use MSTEP register to set microstep resolution
  /* bit: 8 = 0  */ this->write_gconf_multistep_filt(false);
  /* bit: 9 = 0  */ this->write_gconf_test_mode(false);
  /* End of GCONF */

  this->write_register(TMC2209_IHOLD_IRUN, 0x00071703);
  this->write_register(TMC2209_TPOWERDOWN, 0x00000014);
  this->write_register(TMC2209_CHOPCONF, 0x10000053);
  this->write_register(TMC2209_PWMCONF, 0xC10D0024);

  if (this->ottrim_.has_value()) {
    this->write_ottrim(this->ottrim_.value());
  }

  this->write_vactual(0);

#if defined(USE_DIAG_PIN)
  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(ISRStore::pin_isr, &this->diag_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->diag_isr_store_.pin_triggered_ptr = &this->diag_triggered_;
#endif

  this->setup_event_handlers();
  this->high_freq_.start();

  ESP_LOGCONFIG(TAG, "TMC2209 setup done.");
}

void TMC2209::handle_diag_rise_event() {
  this->on_alert_callback_.call(DIAG_TRIGGERED);
  // TODO: Read driver error and broadcast event accordingly
  this->on_alert_callback_.call(STALLED);
}

void TMC2209::setup_event_handlers() {
  this->diag_handler_.set_callbacks(                 // DIAG
      [this]() { this->handle_diag_rise_event(); },  // rise
      nullptr                                        // fall
  );

  this->otpw_handler_.set_callbacks(  // overtemperature prewarning
      [this]() {
        this->on_alert_callback_.call(OVERTEMPERATURE_PREWARNING);
        this->status_set_warning("driver is warning about overheating!");
      },
      [this]() {
        this->on_alert_callback_.call(OVERTEMPERATURE_PREWARNING_CLEARED);
        this->status_clear_warning();
      });

  this->ot_handler_.set_callbacks(  // overtemperature
      [this]() {
        this->on_alert_callback_.call(OVERTEMPERATURE);
        this->status_set_error("driver is overheating!");
      },
      [this]() {
        this->on_alert_callback_.call(OVERTEMPERATURE_CLEARED);
        this->status_clear_error();
      });

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

  this->olb_handler_.set_callbacks(                                     // olb
      [this]() { this->on_alert_callback_.call(B_OPEN_LOAD); },         // rise
      [this]() { this->on_alert_callback_.call(B_OPEN_LOAD_CLEARED); }  // fall
  );

  this->ola_handler_.set_callbacks(                                     // ola
      [this]() { this->on_alert_callback_.call(A_OPEN_LOAD); },         // rise
      [this]() { this->on_alert_callback_.call(A_OPEN_LOAD_CLEARED); }  // fall
  );

  this->s2vsb_handler_.set_callbacks(                                        // s2vsb
      [this]() { this->on_alert_callback_.call(B_LOW_SIDE_SHORT); },         // rise
      [this]() { this->on_alert_callback_.call(B_LOW_SIDE_SHORT_CLEARED); }  // fall
  );

  this->s2vsa_handler_.set_callbacks(                                        // s2vsa
      [this]() { this->on_alert_callback_.call(A_LOW_SIDE_SHORT); },         // rise
      [this]() { this->on_alert_callback_.call(A_LOW_SIDE_SHORT_CLEARED); }  // fall
  );

  this->s2gb_handler_.set_callbacks(                                       // s2gb
      [this]() { this->on_alert_callback_.call(B_GROUND_SHORT); },         // rise
      [this]() { this->on_alert_callback_.call(B_GROUND_SHORT_CLEARED); }  // fall
  );

  this->s2ga_handler_.set_callbacks(                                       // s2ga
      [this]() { this->on_alert_callback_.call(A_GROUND_SHORT); },         // rise
      [this]() { this->on_alert_callback_.call(A_GROUND_SHORT_CLEARED); }  // fall
  );
}

void TMC2209::loop() {
  if (this->is_failed()) {
    return;
  }

  // Emit DIAG event and clear flag if not rasied anymore
#if defined(USE_DIAG_PIN)
  this->diag_handler_.check(this->diag_triggered_);
  if (this->diag_triggered_) {
    this->diag_triggered_ = this->diag_pin_->digital_read();  // clear or keep flag
  }
#else
  this->diag_triggered_ = this->read_ioin_diag();  // clear or keep flag
  this->diag_handler_.check(this->diag_triggered_);
#endif

  const uint32_t drv_status = this->read_drv_status();
  this->ot_handler_.check(static_cast<bool>((drv_status >> 1) & 1));
  this->otpw_handler_.check(static_cast<bool>(drv_status & 1));
  this->t157_handler_.check(static_cast<bool>((drv_status >> 11) & 1));
  this->t150_handler_.check(static_cast<bool>((drv_status >> 10) & 1));
  this->t143_handler_.check(static_cast<bool>((drv_status >> 9) & 1));
  this->t120_handler_.check(static_cast<bool>((drv_status >> 8) & 1));
  this->olb_handler_.check(static_cast<bool>((drv_status >> 7) & 1));
  this->ola_handler_.check(static_cast<bool>((drv_status >> 6) & 1));
  this->s2vsb_handler_.check(static_cast<bool>((drv_status >> 5) & 1));
  this->s2vsa_handler_.check(static_cast<bool>((drv_status >> 4) & 1));
  this->s2gb_handler_.check(static_cast<bool>((drv_status >> 3) & 1));
  this->s2ga_handler_.check(static_cast<bool>((drv_status >> 2) & 1));
}

uint16_t TMC2209::get_microsteps() {
  const uint8_t mres = this->read_chopconf_mres();
  return 256 >> mres;
}

void TMC2209::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->write_chopconf_mres(mres);
}

float TMC2209::get_motor_load() {
  const uint16_t result = this->read_stallguard_sgresult();
  return (510.0 - (float) result) / (510.0 - (float) this->read_stallguard_sgthrs() * 2.0);
}

float TMC2209::rms_current_hold_scale() { return this->rms_current_hold_scale_; }
void TMC2209::rms_current_hold_scale(float scale) {
  this->rms_current_hold_scale_ = scale;
  this->set_rms_current_();
}

void TMC2209::set_rms_current(float A) {
  this->rms_current_ = A;
  this->set_rms_current_();
}

float TMC2209::get_rms_current() { return this->current_scale_to_rms_current_(this->read_ihold_irun_irun()); }

void TMC2209::set_rms_current_() {
  uint8_t current_scale = 32.0 * 1.41421 * this->rms_current_ * (this->rsense_ + 0.02) / 0.325 - 1;

  if (current_scale < 16) {
    this->write_chopconf_vsense(true);
    current_scale = 32.0 * 1.41421 * this->rms_current_ * (this->rsense_ + 0.02) / 0.180 - 1;
  } else {
    this->write_chopconf_vsense(false);
  }

  if (current_scale > 31) {
    current_scale = 31;
    ESP_LOGW(TAG, "Selected rsense has a current limit of %.3f A", this->current_scale_to_rms_current_(current_scale));
  }

  this->write_ihold_irun_irun(current_scale);
  this->write_ihold_irun_ihold(current_scale * this->rms_current_hold_scale_);
}

float TMC2209::current_scale_to_rms_current_(uint8_t current_scaling) {
  return (current_scaling + 1) / 32.0 * (this->read_chopconf_vsense() ? 0.180 : 0.325) / (this->rsense_ + 0.02) /
         sqrtf(2);
}

void TMC2209::ihold_irun_ihold_delay_ms(uint32_t delay_in_ms) {
  this->write_ihold_irun_ihold_delay(((float) delay_in_ms / 262144.0) * ((float) this->clock_frequency_ / 1000.0));
}

uint32_t TMC2209::ihold_irun_ihold_delay_ms() {
  return (this->read_ihold_irun_ihold_delay() * 262144) / (this->clock_frequency_ / 1000);
}

void TMC2209::tpowerdown_ms(uint32_t delay_in_ms) {
  this->write_tpowerdown(((float) delay_in_ms / 262144.0) * ((float) this->clock_frequency_ / 1000.0));
};

uint32_t TMC2209::tpowerdown_ms() { return (this->read_tpowerdown() * 262144) / (this->clock_frequency_ / 1000); };

std::tuple<uint8_t, uint8_t> TMC2209::unpack_ottrim_values(uint8_t ottrim) {
  switch (ottrim) {
    case 0b00:
      return std::make_tuple(120, 143);
      break;
    case 0b01:
      return std::make_tuple(120, 150);
      break;
    case 0b10:
      return std::make_tuple(143, 150);
      break;
    case 0b11:
      return std::make_tuple(143, 157);
      break;
  }
  return std::make_tuple(0, 0);
}

/***
 * TMC-API wrappers
 *
 */

extern "C" {
bool tmc2209_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209 *comp = components[id];

  optional<bool> ok;

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    ok = comp->read_array(data, writeLength);
    comp->flush();
  }

  if (readLength > 0) {
    ok = comp->read_array(data, readLength);
  }

  // if (!ok.value()) {
  //   comp->status_set_error(("Failed to communicate with driver"));
  // } else {
  //   if (comp->status_has_error()) {
  //     comp->status_clear_error();
  //   }
  // }

  return ok.value_or(false);
}

uint8_t tmc2209_getNodeAddress(uint16_t id) {
  TMC2209 *comp = components[id];
  return comp->get_address();
}
}

/** setters */
void TMC2209::write_gconf_iscale_analog(bool use_vref) { this->write_field(TMC2209_I_SCALE_ANALOG_FIELD, use_vref); }
void TMC2209::write_gconf_internal_rsense(bool internal) { this->write_field(TMC2209_INTERNAL_RSENSE_FIELD, internal); }
void TMC2209::write_gconf_en_spreadcycle(bool enable) { this->write_field(TMC2209_EN_SPREADCYCLE_FIELD, enable); }
void TMC2209::write_gconf_shaft(bool inverse) { this->write_field(TMC2209_SHAFT_FIELD, inverse); }
void TMC2209::write_gconf_index_otpw(bool use_otpw) { this->write_field(TMC2209_INDEX_OTPW_FIELD, use_otpw); }
void TMC2209::write_gconf_index_step(bool enable) { this->write_field(TMC2209_INDEX_STEP_FIELD, enable); }
void TMC2209::write_gconf_pdn_disable(bool disable) { this->write_field(TMC2209_PDN_DISABLE_FIELD, disable); }
void TMC2209::write_gconf_mstep_reg_select(bool use) { this->write_field(TMC2209_MSTEP_REG_SELECT_FIELD, use); }
void TMC2209::write_gconf_multistep_filt(bool enable) { this->write_field(TMC2209_MULTISTEP_FILT_FIELD, enable); }
void TMC2209::write_gconf_test_mode(bool enable) { this->write_field(TMC2209_TEST_MODE_FIELD, enable); }
void TMC2209::write_vactual(int32_t velocity) { this->write_field(TMC2209_VACTUAL_FIELD, velocity); }
void TMC2209::write_ihold_irun_ihold(uint8_t current) { this->write_field(TMC2209_IRUN_FIELD, current); }
void TMC2209::write_ihold_irun_irun(uint8_t current) { this->write_field(TMC2209_IHOLD_FIELD, current); }
void TMC2209::write_stallguard_sgthrs(uint8_t threshold) { this->write_register(TMC2209_SGTHRS, (int32_t) threshold); }
void TMC2209::write_coolstep_tcoolthrs(int32_t threshold) { this->write_register(TMC2209_TCOOLTHRS, threshold); }
void TMC2209::write_chopconf_mres(uint8_t index) { this->write_field(TMC2209_MRES_FIELD, index); }
void TMC2209::write_chopconf_intpol(bool enable) { this->write_field(TMC2209_INTPOL_FIELD, enable); }
void TMC2209::write_ottrim(uint8_t ottrim) { this->write_field(TMC2209_OTTRIM_FIELD, ottrim); }
void TMC2209::write_chopconf_vsense(bool high_sensitivity) {
  this->write_field(TMC2209_VSENSE_FIELD, high_sensitivity);
}
void TMC2209::write_tpowerdown(uint8_t factor) { this->write_field(TMC2209_TPOWERDOWN_FIELD, factor); }
void TMC2209::write_ihold_irun_ihold_delay(uint8_t factor) {
  if (factor > 15) {
    ESP_LOGW(TAG, "IHOLDDELAY is limited to 15. This is the raw value and not the delay in microseconds.");
    factor = 15;
  }
  this->write_field(TMC2209_IHOLDDELAY_FIELD, factor);
}

/** getters */
bool TMC2209::read_gconf_index_otpw() { return this->read_field(TMC2209_INDEX_OTPW_FIELD); }
bool TMC2209::read_ioin_diag() { return this->read_field(TMC2209_DIAG_FIELD); }
uint32_t TMC2209::read_drv_status() { return this->read_register(TMC2209_DRV_STATUS); }
bool TMC2209::read_chopconf_vsense() { return this->read_field(TMC2209_VSENSE_FIELD); }
int8_t TMC2209::read_ioin_chip_version() { return this->read_field(TMC2209_VERSION_FIELD); }
uint8_t TMC2209::read_ihold_irun_irun() { return this->read_field(TMC2209_IHOLD_FIELD); }
uint8_t TMC2209::read_ihold_irun_ihold_delay() { return this->read_field(TMC2209_IHOLDDELAY_FIELD); }
uint8_t TMC2209::read_stallguard_sgthrs() { return this->read_register(TMC2209_SGTHRS); }
uint8_t TMC2209::read_chopconf_mres() { return this->read_field(TMC2209_MRES_FIELD); }
uint8_t TMC2209::read_tpowerdown() { return this->read_field(TMC2209_TPOWERDOWN_FIELD); }
uint16_t TMC2209::read_stallguard_sgresult() { return this->read_register(TMC2209_SG_RESULT); }
uint8_t TMC2209::read_ottrim() { return this->read_field(TMC2209_OTTRIM_FIELD); }

}  // namespace tmc2209
}  // namespace esphome
