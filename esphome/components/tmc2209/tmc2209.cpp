#include "tmc2209.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

static const char *TAG = "tmc2209";

TMC2209::TMC2209(uint8_t address, uint32_t clock_frequency) : address_(address), clock_frequency_(clock_frequency) {
  this->id_ = component_index++;
  components[this->id_] = this;
}

/** TMC-API wrappers **/
extern "C" bool tmc2209_readWriteUART(uint16_t id, uint8_t *data, size_t writeLength, size_t readLength) {
  TMC2209 *comp = components[id];
  optional<bool> ok;

  if (writeLength > 0) {
    comp->write_array(data, writeLength);

    // chop off transmitted data from the rx buffer and flush due to one-wire uart filling up rx when transmitting
    ok = comp->read_array(data, writeLength);
    comp->flush();
  }

  // TODO: maybe do something with IFCNT for write verification
  if (readLength > 0) {
    ok = comp->read_array(data, readLength);
  }
  return ok.value_or(false);
}

extern "C" uint8_t tmc2209_getNodeAddress(uint16_t id) {
  TMC2209 *comp = components[id];
  return comp->get_address();
}

void TMC2209::write_register(uint8_t address, int32_t value) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "driver failed at some point.");
    return;
  }
  tmc2209_writeRegister(this->id_, address, value);
}

int32_t TMC2209::read_register(uint8_t address) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "driver failed at some point.");
    return 0;
  }
  return tmc2209_readRegister(this->id_, address);
}

void TMC2209::write_field(RegisterField field, uint32_t value) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "driver failed at some point.");
    return;
  }
  tmc2209_fieldWrite(this->id_, field, value);
}

uint32_t TMC2209::read_field(RegisterField field) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "driver failed at some point.");
    return 0;
  }
  return tmc2209_fieldRead(this->id_, field);
}

/** End of TMC-API wrappers **/

void TMC2209::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Stepper...");

  this->high_freq_.start();

  while (!this->is_ready()) {
  };

  if (!this->read_field(TMC2209_VERSION_FIELD)) {
    this->status_set_error("Failed to communicate with driver");
    this->mark_failed();
  }

  if (this->enn_pin_ != nullptr) {
    this->enn_pin_->setup();
    this->enable(true);
  }

  this->write_field(TMC2209_VACTUAL_FIELD, 0);
  this->write_field(TMC2209_I_SCALE_ANALOG_FIELD, false);
  this->write_field(TMC2209_INTERNAL_RSENSE_FIELD, false);
  this->write_field(TMC2209_EN_SPREADCYCLE_FIELD, false);
  this->write_field(TMC2209_SHAFT_FIELD, false);
  this->write_field(TMC2209_INDEX_OTPW_FIELD, true);
  this->write_field(TMC2209_INDEX_STEP_FIELD, false);
  this->write_field(TMC2209_PDN_DISABLE_FIELD, true);
  this->write_field(TMC2209_MSTEP_REG_SELECT_FIELD, true);
  this->write_field(TMC2209_MULTISTEP_FILT_FIELD, false);
  this->write_field(TMC2209_TEST_MODE_FIELD, false);
  // this->write_field(TMC2209_PWM_SCALE_AUTO_FIELD, 1);
  // this->write_field(TMC2209_PWM_GRAD_AUTO_FIELD, 1);

  this->write_field(TMC2209_FREEWHEEL_FIELD, 1);
  this->write_field(TMC2209_IHOLD_FIELD, 0);

#if defined(USE_UART_CONTROL)
  /* Configure INDEX for pulse feedback from the driver */
  // Check mux from figure 15.1 from datasheet rev1.09
  this->write_field(TMC2209_INDEX_STEP_FIELD, true);
  this->write_field(TMC2209_INDEX_OTPW_FIELD, false);
  this->ips_.target_position_ptr = &this->target_position;
  this->ips_.current_position_ptr = &this->current_position;
  this->ips_.direction_ptr = &this->direction_;
  this->index_pin_->setup();
  this->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_, gpio::INTERRUPT_RISING_EDGE);
#endif

#if defined(USE_PULSE_CONTROL)
  this->step_pin_->setup();
  this->step_pin_->digital_write(false);
  this->dir_pin_->setup();
  this->dir_pin_->digital_write(false);
#endif

  // vsense
  if (this->vsense_.has_value())
    this->write_field(TMC2209_VSENSE_FIELD, this->vsense_.value());
  this->vsense_ = this->read_field(TMC2209_VSENSE_FIELD);

  // ottrim
  if (this->ottrim_.has_value())
    this->write_field(TMC2209_OTTRIM_FIELD, this->ottrim_.value());
  this->ottrim_ = this->read_field(TMC2209_OTTRIM_FIELD);

#if defined(ENABLE_DRIVER_ALERT_EVENTS)
#if defined(USE_DIAG_PIN)
  // dont bother configuring diag pin as it is only used for driver alerts
  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(ISRStore::pin_isr, &this->diag_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->diag_isr_store_.pin_triggered_ptr = &this->diag_isr_triggered_;

  this->diag_handler_.set_callbacks(                                     // DIAG
      [this]() { this->on_alert_callback_.call(DIAG_TRIGGERED); },       // rise
      [this]() { this->on_alert_callback_.call(DIAG_TRIGGER_CLEARED); }  // fall
  );
#else
  // poll driver every x time to check for errors
  this->set_interval(POLL_STATUS_INTERVAL, [this] { this->check_driver_status_(); });
#endif

  this->stalled_handler_.set_on_rise_callback([this]() { this->on_alert_callback_.call(STALLED); });  // stalled

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

  this->uvcp_handler_.set_callbacks(                                        // uc_vp
      [this]() { this->on_alert_callback_.call(CP_UNDERVOLTAGE); },         // rise
      [this]() { this->on_alert_callback_.call(CP_UNDERVOLTAGE_CLEARED); }  // fall
  );

#endif

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209::check_driver_status_() {
#if defined(ENABLE_DRIVER_ALERT_EVENTS)
  const uint32_t drv_status = this->read_register(TMC2209_DRV_STATUS);
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
  this->uvcp_handler_.check(this->read_field(TMC2209_UV_CP_FIELD));
#endif
}

void TMC2209::loop() {
  /** Alert events **/
#if defined(ENABLE_DRIVER_ALERT_EVENTS)
  if (this->current_speed_ == this->max_speed_) {
    this->stalled_handler_.check(this->is_stalled());
  }
#if defined(USE_DIAG_PIN)
  if (this->diag_isr_triggered_) {
    this->diag_handler_.check(this->diag_isr_triggered_);
    this->stalled_handler_.check(this->is_stalled());
    this->check_driver_status_();

    // keep polling status as long as this DIAG is up
    this->diag_isr_triggered_ = this->diag_pin_->digital_read();
  }
#endif
#endif
  /** */

  /** Serial control handling **/
#if defined(USE_UART_CONTROL)
  // Compute and set speed and direction to move the motor in
  const int32_t to_target = (this->target_position - this->current_position);
  this->direction_ = (to_target != 0 ? (Direction) (to_target / abs(to_target)) : Direction::NONE);  // yield 1, -1 or 0

  this->calculate_speed_(micros());
  // -2.8 magically to synchronizes feedback with set velocity
  const uint32_t velocity = ((int8_t) this->direction_) * this->current_speed_ * -2.8;
  if (this->read_field(TMC2209_VACTUAL_FIELD) != velocity) {
    this->write_field(TMC2209_VACTUAL_FIELD, velocity);
  }

#endif
  /** */

/** Pulse train handling **/
#if defined(USE_PULSE_CONTROL)
  // Compute and set speed and direction to move the motor in
  this->direction_ = (Direction) this->should_step_();
  if (this->direction_ != Direction::NONE) {
    this->dir_pin_->digital_write(this->direction_ == Direction::CLOCKWISE);
    delayMicroseconds(5);
    this->step_pin_->digital_write(true);
    delayMicroseconds(5);
    this->step_pin_->digital_write(false);
  }
#endif
  /** */
}

bool TMC2209::is_stalled() {
  const auto sgthrs = this->read_register(TMC2209_SGTHRS);
  const auto sgresult = this->read_register(TMC2209_SG_RESULT);
  return (2 * sgthrs) > sgresult;
}

uint16_t TMC2209::get_microsteps() {
  const uint8_t mres = this->read_field(TMC2209_MRES_FIELD);
  return 256 >> mres;
}

void TMC2209::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 8; mres > 0; mres--)
    if ((256 >> mres) == ms)
      return this->write_field(TMC2209_MRES_FIELD, mres);
}

float TMC2209::get_motor_load() {
  const uint16_t result = this->read_register(TMC2209_SG_RESULT);
  return (510.0 - (float) result) / (510.0 - this->read_register(TMC2209_SGTHRS) * 2.0);
}

float TMC2209::read_vsense() { return (this->read_field(TMC2209_VSENSE_FIELD) ? VSENSE_LOW : VSENSE_HIGH); }

uint16_t TMC2209::current_scale_to_rms_current_mA(uint8_t cs) {
  cs = clamp<uint8_t>(cs, 0, 31);
  if (cs == 0)
    return 0;
  return (cs + 1) / 32.0f * this->read_vsense() / (this->rsense_.value_or(0) + 0.02f) * (1 / sqrtf(2)) * 1000.0f;
}

uint8_t TMC2209::rms_current_to_current_scale_mA(uint16_t mA) {
  if (mA == 0) {
    return 0;
  }
  const uint8_t cs = 32.0f * sqrtf(2) * mA2A(mA) * ((this->rsense_.value_or(0) + 0.02f) / this->read_vsense()) - 1.0f;
  return clamp<uint8_t>(cs, 0, 31);
}

void TMC2209::write_run_current_mA(uint16_t mA) {
  const uint8_t cs = this->rms_current_to_current_scale_mA(mA);
  this->write_field(TMC2209_IRUN_FIELD, cs);
}

uint16_t TMC2209::read_run_current_mA() {
  const uint8_t cs = this->read_field(TMC2209_IRUN_FIELD);
  return this->current_scale_to_rms_current_mA(cs);
}

void TMC2209::write_hold_current_mA(uint16_t mA) {
  const uint8_t cs = this->rms_current_to_current_scale_mA(mA);
  this->write_field(TMC2209_IHOLD_FIELD, cs);
}

uint16_t TMC2209::read_hold_current_mA() {
  const uint8_t cs = this->read_field(TMC2209_IHOLD_FIELD);
  return this->current_scale_to_rms_current_mA(cs);
}

void TMC2209::set_tpowerdown_ms(uint32_t delay_in_ms) {
  auto tpowerdown = ((float) delay_in_ms / 262144.0) * ((float) this->clock_frequency_ / 1000.0);
  this->write_field(TMC2209_TPOWERDOWN_FIELD, tpowerdown);
};

uint32_t TMC2209::get_tpowerdown_ms() {
  return (this->read_field(TMC2209_TPOWERDOWN_FIELD) * 262144.0) / (this->clock_frequency_ / 1000.0);
};

std::tuple<uint8_t, uint8_t> TMC2209::unpack_ottrim_values(uint8_t ottrim) {
  switch (ottrim) {
    case 0b00:
      return std::make_tuple(120, 143);
    case 0b01:
      return std::make_tuple(120, 150);
    case 0b10:
      return std::make_tuple(143, 150);
    case 0b11:
      return std::make_tuple(143, 157);
  }
  return std::make_tuple(0, 0);
}

void TMC2209::set_target(int32_t steps) {
  if (this->enn_pin_ != nullptr) {
    this->enable(true);
  }

  stepper::Stepper::set_target(steps);
}

void TMC2209::stop() {
  stepper::Stepper::stop();
  this->direction_ = Direction::NONE;

#if defined(USE_UART_CONTROL)
  this->write_field(TMC2209_VACTUAL_FIELD, 0);
#endif
}

void TMC2209::enable(bool enable) {
  if (!enable) {
    this->stop();
  }

  if (this->enn_pin_ == nullptr) {
    return ESP_LOGW(TAG, "enn_pin is not configured and setting enable/disable has no effect");
  }

  if (this->enn_pin_state_ != !enable) {
    this->enn_pin_state_ = !enable;
    this->enn_pin_->digital_write(this->enn_pin_state_);
  }
}

void TMC2209::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");

#if defined(USE_UART_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: UART with feedback");
#elif defined(USE_PULSE_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: Pulse train");
#else
  ESP_LOGE(TAG, "No control method defined!");
#endif
  LOG_STEPPER(this);

  LOG_PIN("  ENN Pin: ", this->enn_pin_);
  LOG_PIN("  DIAG Pin: ", this->diag_pin_);
  LOG_PIN("  INDEX Pin: ", this->index_pin_);
  LOG_PIN("  STEP Pin: ", this->step_pin_);
  LOG_PIN("  DIR Pin: ", this->dir_pin_);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps());

  // Read IC version
  const int8_t icv_ = this->read_field(TMC2209_VERSION_FIELD);
  if (std::isnan(icv_) || icv_ == 0) {
    ESP_LOGE(TAG, "  Unable to read IC version. Is the driver powered and wired correctly?");
  } else if (icv_ == TMC2209_IC_VERSION_33) {
    ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", icv_);
  } else {
    ESP_LOGE(TAG, "  Detected unknown IC version: 0x%02X", icv_);
  }

  auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(TMC2209_OTTRIM_FIELD));  // c++17 required
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot);
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz", this->clock_frequency_);
  ESP_LOGCONFIG(TAG, "  Driver status poll interval: %dms", POLL_STATUS_INTERVAL);

  ESP_LOGCONFIG(TAG, "  Register dump:");
  ESP_LOGCONFIG(TAG, "    GCONF: 0x%X", this->read_register(TMC2209_GCONF));
  ESP_LOGCONFIG(TAG, "    GSTAT: 0x%X", this->read_register(TMC2209_GSTAT));
  ESP_LOGCONFIG(TAG, "    IFCNT: 0x%X", this->read_register(TMC2209_IFCNT));
  ESP_LOGCONFIG(TAG, "    SLAVECONF: 0x%X", this->read_register(TMC2209_SLAVECONF));
  ESP_LOGCONFIG(TAG, "    OTP_PROG: 0x%X", this->read_register(TMC2209_OTP_PROG));
  ESP_LOGCONFIG(TAG, "    OTP_READ: 0x%X", this->read_register(TMC2209_OTP_READ));
  ESP_LOGCONFIG(TAG, "    IOIN: 0x%X", this->read_register(TMC2209_IOIN));
  ESP_LOGCONFIG(TAG, "    FACTORY_CONF: 0x%X", this->read_register(TMC2209_FACTORY_CONF));
  ESP_LOGCONFIG(TAG, "    IHOLD_IRUN: 0x%X", this->read_register(TMC2209_IHOLD_IRUN));
  ESP_LOGCONFIG(TAG, "    TPOWERDOWN: 0x%X", this->read_register(TMC2209_TPOWERDOWN));
  ESP_LOGCONFIG(TAG, "    TSTEP: 0x%X", this->read_register(TMC2209_TSTEP));
  ESP_LOGCONFIG(TAG, "    TPWMTHRS: 0x%X", this->read_register(TMC2209_TPWMTHRS));
  ESP_LOGCONFIG(TAG, "    TCOOLTHRS: 0x%X", this->read_register(TMC2209_TCOOLTHRS));
  ESP_LOGCONFIG(TAG, "    VACTUAL: 0x%X", this->read_register(TMC2209_VACTUAL));
  ESP_LOGCONFIG(TAG, "    SGTHRS: 0x%X", this->read_register(TMC2209_SGTHRS));
  ESP_LOGCONFIG(TAG, "    SG_RESULT: 0x%X", this->read_register(TMC2209_SG_RESULT));
  ESP_LOGCONFIG(TAG, "    COOLCONF: 0x%X", this->read_register(TMC2209_COOLCONF));
  ESP_LOGCONFIG(TAG, "    MSCNT: 0x%X", this->read_register(TMC2209_MSCNT));
  ESP_LOGCONFIG(TAG, "    MSCURACT: 0x%X", this->read_register(TMC2209_MSCURACT));
  ESP_LOGCONFIG(TAG, "    CHOPCONF: 0x%X", this->read_register(TMC2209_CHOPCONF));
  ESP_LOGCONFIG(TAG, "    DRV_STATUS: 0x%X", this->read_register(TMC2209_DRV_STATUS));
  ESP_LOGCONFIG(TAG, "    PWMCONF: 0x%X", this->read_register(TMC2209_PWMCONF));
  ESP_LOGCONFIG(TAG, "    PWMSCALE: 0x%X", this->read_register(TMC2209_PWMSCALE));
  ESP_LOGCONFIG(TAG, "    PWM_AUTO: 0x%X", this->read_register(TMC2209_PWM_AUTO));
}

}  // namespace tmc2209
}  // namespace esphome
