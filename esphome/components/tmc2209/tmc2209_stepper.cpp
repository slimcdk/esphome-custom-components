#include "tmc2209_stepper.h"
#include "tmc2209_api_registers.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

void TMC2209Stepper::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Stepper...");

  this->high_freq_.start();

  if (!this->read_field(VERSION_FIELD)) {
    this->status_set_error("Failed to communicate with driver");
    this->mark_failed();
  }

  this->write_field(PDN_DISABLE_FIELD, true);
  this->write_field(VACTUAL_FIELD, 0);
  this->write_field(INTERNAL_RSENSE_FIELD, INTERNAL_RSENSE);
  this->write_field(I_SCALE_ANALOG_FIELD, false);
  this->write_field(SHAFT_FIELD, false);
  this->write_field(MSTEP_REG_SELECT_FIELD, true);
  // this->write_field(MULTISTEP_FILT_FIELD, false);
  this->write_field(TEST_MODE_FIELD, false);
  // this->write_field(PWM_SCALE_AUTO_FIELD, 1);
  // this->write_field(PWM_GRAD_AUTO_FIELD, 1);

#if defined(USE_UART_CONTROL)
  /* Configure INDEX for pulse feedback from the driver */
  // Check mux from figure 15.1 from datasheet rev1.09
  this->write_field(INDEX_STEP_FIELD, true);
  this->write_field(INDEX_OTPW_FIELD, false);
  this->ips_.current_position_ptr = &this->current_position;
  this->ips_.direction_ptr = &this->current_direction_;
  this->index_pin_->setup();
  this->index_pin_->attach_interrupt(IndexPulseStore::pulse_isr, &this->ips_, gpio::INTERRUPT_RISING_EDGE);
#endif

#if defined(USE_PULSE_CONTROL)

  // TODO: Test if below is beneficial to have
  // this->write_field(MULTISTEP_FILT_FIELD, false);
  // this->write_field(DEDGE_FIELD, true);

  this->step_pin_->setup();
  this->step_pin_->digital_write(false);
  this->dir_pin_->setup();
  this->dir_pin_->digital_write(false);
#endif

#if defined(VSENSE)
  this->write_field(VSENSE_FIELD, VSENSE);
#endif

#if defined(OTTRIM)
  this->write_field(OTTRIM_FIELD, OTTRIM);
#endif

#if defined(ENABLE_DRIVER_ALERT_EVENTS)
#if defined(USE_DIAG_PIN)
  // dont bother configuring diag pin as it is only used for driver alerts
  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(ISRPinTriggerStore::pin_isr, &this->diag_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->diag_isr_store_.pin_triggered_ptr = &this->diag_triggered_;

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

  if (this->enn_pin_ != nullptr) {
    this->enn_pin_->setup();
  }
  this->enable(true);

  ESP_LOGCONFIG(TAG, "TMC2209 Stepper setup done.");
}

void TMC2209Stepper::loop() {
  /** Alert events **/
#if defined(ENABLE_DRIVER_ALERT_EVENTS)

  const bool standstill = this->read_field(STST_FIELD);
  if (!standstill && this->current_speed_ >= (this->max_speed_ * this->sdal_)) {
    this->stalled_handler_.check(this->is_stalled());
  }
#if defined(USE_DIAG_PIN)
  if (this->diag_triggered_) {
    this->diag_handler_.check(this->diag_triggered_);
    this->stalled_handler_.check(this->is_stalled());
    this->check_driver_status_();

    // keep polling status as long as this DIAG is up
    this->diag_triggered_ = this->diag_pin_->digital_read();
  }
#endif
#endif
  /** */

  /** Serial control handling **/
#if defined(USE_UART_CONTROL)
  // Compute and set speed and direction to move the motor in
  const int32_t to_target = (this->target_position - this->current_position);
  this->current_direction_ = (to_target != 0 ? (stepper::Direction)(to_target / abs(to_target))
                                             : stepper::Direction::STANDSTILL);  // yield 1, -1 or 0

  this->calculate_speed_(micros());
  // -2.8 magically to synchronizes feedback with set velocity
  const uint32_t velocity = ((int8_t) this->current_direction_) * this->current_speed_ * -2.8;
  if (this->read_field(VACTUAL_FIELD) != velocity) {
    this->write_field(VACTUAL_FIELD, velocity);
  }

#endif
  /** */

/** Pulse train handling **/
#if defined(USE_PULSE_CONTROL)
  // Compute and set speed and direction to move the motor in
  this->current_direction_ = (Direction) this->should_step_();
  if (this->current_direction_ != Direction::STANDSTILL) {
    this->dir_pin_->digital_write(this->current_direction_ == Direction::FORWARD);
    delayMicroseconds(5);
    this->step_pin_->digital_write(true);
    delayMicroseconds(5);
    this->step_pin_->digital_write(false);
  }
#endif
  /** */
}

void TMC2209Stepper::set_target(int32_t steps) {
  if (!this->is_enabled_) {
    this->enable(true);
  }

  stepper::Stepper::set_target(steps);
}

void TMC2209Stepper::stop() {
  stepper::Stepper::stop();

#if defined(USE_UART_CONTROL)
  this->write_field(VACTUAL_FIELD, 0);
#endif
}

void TMC2209Stepper::enable(bool enable) {
  if (!enable) {
    this->stop();
  }

  if (this->enn_pin_ != nullptr) {
    this->enn_pin_->digital_write(!enable);
  } else {
    this->write_field(TOFF_FIELD, enable ? 3 : 0);
  }
  this->is_enabled_ = enable;
}

void TMC2209Stepper::dump_config() {
  ESP_LOGCONFIG(TAG, "TMC2209 Stepper:");

#if defined(USE_UART_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: Over UART with feedback on INDEX");
#elif defined(USE_PULSE_CONTROL)
  ESP_LOGCONFIG(TAG, "  Control: Pulses on STEP and direction on DIR");
#else
  ESP_LOGE(TAG, "No control method defined!");
#endif
  LOG_STEPPER(this);

  // Read IC version
  const int8_t icv_ = this->read_field(VERSION_FIELD);
  if (std::isnan(icv_) || icv_ == 0) {
    ESP_LOGE(TAG, "  Unable to read IC version. Is the driver powered and wired correctly?");
  } else if (icv_ == IC_VERSION_33) {
    ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", icv_);
  } else {
    ESP_LOGE(TAG, "  Detected unknown IC version: 0x%02X", icv_);
  }

  if (this->enn_pin_) {
    LOG_PIN("  ENN Pin: ", this->enn_pin_);
  } else {
    ESP_LOGCONFIG(TAG, "  Enable/disable driver with TOFF");
  }

  LOG_PIN("  DIAG Pin: ", this->diag_pin_);
  if (!this->diag_pin_) {
    ESP_LOGCONFIG(TAG, "  Driver status poll interval: %dms", POLL_STATUS_INTERVAL);
  }
  LOG_PIN("  INDEX Pin: ", this->index_pin_);
  LOG_PIN("  STEP Pin: ", this->step_pin_);
  LOG_PIN("  DIR Pin: ", this->dir_pin_);

  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->driver_address_);
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps());

  ESP_LOGCONFIG(TAG, "  Currents:");
  if (this->read_field(INTERNAL_RSENSE_FIELD)) {
    ESP_LOGCONFIG(TAG, "    RSense: %.3f Ohm (internal RDSon value)", RSENSE);
  } else {
    ESP_LOGCONFIG(TAG, "    RSense: %.3f Ohm (external sense resistors)", RSENSE);
  }

  if (this->read_field(VSENSE_FIELD)) {
    ESP_LOGCONFIG(TAG, "    VSense: True (low heat dissipation)");
  } else {
    ESP_LOGCONFIG(TAG, "    VSense: False (high heat dissipation)");
  }

  ESP_LOGCONFIG(TAG, "    Currently set IRUN: %d (%d mA)", this->read_field(IRUN_FIELD), this->read_run_current_mA());
  ESP_LOGCONFIG(TAG, "    Currently set IHOLD: %d (%d mA)", this->read_field(IHOLD_FIELD),
                this->read_hold_current_mA());
  ESP_LOGCONFIG(TAG, "    Maximum allowable: %d mA", this->current_scale_to_rms_current_mA(31));

  const auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(OTTRIM_FIELD));  // c++17 required
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot);
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz", this->clk_frequency_);
  ESP_LOGCONFIG(TAG, "  Active stall detection: At %d steps/s (%.0f%% of max speed)",
                (int) (this->max_speed_ * this->sdal_), (this->sdal_ * 100));

  ESP_LOGCONFIG(TAG, "  Register dump:");
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "GCONF:", this->read_register(GCONF));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "GSTAT:", this->read_register(GSTAT));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IFCNT:", this->read_register(IFCNT));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SLAVECONF:", this->read_register(SLAVECONF));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "OTP_PROG:", this->read_register(OTP_PROG));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "OTP_READ:", this->read_register(OTP_READ));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IOIN:", this->read_register(IOIN));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "FACTORY_CONF:", this->read_register(FACTORY_CONF));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IHOLD_IRUN:", this->read_register(IHOLD_IRUN));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TPOWERDOWN:", this->read_register(TPOWERDOWN));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TSTEP:", this->read_register(TSTEP));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TPWMTHRS:", this->read_register(TPWMTHRS));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TCOOLTHRS:", this->read_register(TCOOLTHRS));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "VACTUAL:", this->read_register(VACTUAL));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SGTHRS:", this->read_register(SGTHRS));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SG_RESULT:", this->read_register(SG_RESULT));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "COOLCONF:", this->read_register(COOLCONF));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "MSCNT:", this->read_register(MSCNT));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "MSCURACT:", this->read_register(MSCURACT));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "CHOPCONF:", this->read_register(CHOPCONF));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "DRV_STATUS:", this->read_register(DRV_STATUS));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWMCONF:", this->read_register(PWMCONF));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWMSCALE:", this->read_register(PWMSCALE));
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWM_AUTO:", this->read_register(PWM_AUTO));
}

}  // namespace tmc2209
}  // namespace esphome
