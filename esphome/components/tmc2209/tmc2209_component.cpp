#include "tmc2209_api_registers.h"
#include "tmc2209_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

// static const char *TAG = "tmc2209";

void TMC2209Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC2209 Component...");

  this->high_freq_.start();

  if (!this->read_field(VERSION_FIELD)) {
    this->status_set_error("Failed to communicate with driver");
    this->mark_failed();
  }

  this->write_field(PDN_DISABLE_FIELD, true);
  this->write_field(INTERNAL_RSENSE_FIELD, this->internal_rsense_);
  this->write_field(I_SCALE_ANALOG_FIELD, false);
  this->write_field(SHAFT_FIELD, false);
  this->write_field(MSTEP_REG_SELECT_FIELD, true);
  this->write_field(TEST_MODE_FIELD, false);

#if defined(VSENSE)
  this->write_field(VSENSE_FIELD, VSENSE);
#endif

#if defined(OTTRIM)
  this->write_field(OTTRIM_FIELD, OTTRIM);
#endif

#if defined(HAS_ENN_PIN)
  this->enn_pin_->setup();
#endif

#if defined(HAS_DIAG_PIN)
  this->diag_pin_->setup();
  this->diag_pin_->attach_interrupt(ISRPinTriggerStore::pin_isr, &this->diag_isr_store_, gpio::INTERRUPT_RISING_EDGE);
  this->diag_isr_store_.pin_triggered_ptr = &this->diag_triggered_;

  this->diag_handler_.set_callbacks(                                             // DIAG
      [this]() { this->on_driver_status_callback_.call(DIAG_TRIGGERED); },       // rise
      [this]() { this->on_driver_status_callback_.call(DIAG_TRIGGER_CLEARED); }  // fall
  );
#endif

#if defined(HAS_INDEX_PIN)
  this->index_pin_->setup();
#endif

#if defined(HAS_STEP_PIN)
  this->step_pin_->setup();
#endif

#if defined(HAS_DIR_PIN)
  this->dir_pin_->setup();
#endif

#if defined(ENABLE_DRIVER_EVENT_EVENTS)
  this->otpw_handler_.set_callbacks(  // overtemperature prewarning
      [this]() {
        this->on_driver_status_callback_.call(OVERTEMPERATURE_PREWARNING);
        this->status_set_warning("driver is overheating soon!");
      },
      [this]() {
        this->on_driver_status_callback_.call(OVERTEMPERATURE_PREWARNING_CLEARED);
        this->status_clear_warning();
      });

  this->ot_handler_.set_callbacks(  // overtemperature
      [this]() {
        this->on_driver_status_callback_.call(OVERTEMPERATURE);
        this->status_set_error("driver is overheating!");
      },
      [this]() {
        this->on_driver_status_callback_.call(OVERTEMPERATURE_CLEARED);
        this->status_clear_error();
      });

  this->t120_handler_.set_callbacks(                                                // t120 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_120C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_120C); }   // fall
  );

  this->t143_handler_.set_callbacks(                                                // t143 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_143C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_143C); }   // fall
  );

  this->t150_handler_.set_callbacks(                                                // t150 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_150C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_150C); }   // fall
  );

  this->t157_handler_.set_callbacks(                                                // t157 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_157C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_157C); }   // fall
  );

  this->ola_handler_.set_callbacks(  // ola
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD);
        this->on_driver_status_callback_.call(OPEN_LOAD_A);
      },
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD_CLEARED);
        this->on_driver_status_callback_.call(OPEN_LOAD_A_CLEARED);
      });

  this->olb_handler_.set_callbacks(  // olb
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD);
        this->on_driver_status_callback_.call(OPEN_LOAD_B);
      },
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD_CLEARED);
        this->on_driver_status_callback_.call(OPEN_LOAD_B_CLEARED);
      });

  this->s2vsa_handler_.set_callbacks(  // s2vsa
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_A);
      },
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_CLEARED);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_A_CLEARED);
      });

  this->s2vsb_handler_.set_callbacks(  // s2vsb
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_B);
      },
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_CLEARED);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_B_CLEARED);
      });

  this->s2ga_handler_.set_callbacks(  // s2ga
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT);
        this->on_driver_status_callback_.call(GROUND_SHORT_A);
      },
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT_CLEARED);
        this->on_driver_status_callback_.call(GROUND_SHORT_A_CLEARED);
      });

  this->s2gb_handler_.set_callbacks(  // s2gb
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT);
        this->on_driver_status_callback_.call(GROUND_SHORT_B);
      },
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT_CLEARED);
        this->on_driver_status_callback_.call(GROUND_SHORT_B_CLEARED);
      });

  this->uvcp_handler_.set_callbacks(  // uc_vp
      [this]() {
        this->on_driver_status_callback_.call(CP_UNDERVOLTAGE);
        this->status_set_warning("undervoltage detected!");
      },  // rise
      [this]() {
        this->on_driver_status_callback_.call(CP_UNDERVOLTAGE_CLEARED);
        this->status_clear_warning();
      }  // fall
  );

  this->set_interval(POLL_DRIVER_STATUS_INTERVAL, [this] { this->poll_driver_status_(); });
#endif

  ESP_LOGCONFIG(TAG, "TMC2209 Component setup done.");
}

void TMC2209Component::loop() {
#if defined(HAS_DIAG_PIN)
  if (this->diag_triggered_) {
    this->diag_triggered_ = this->diag_pin_->digital_read();  // don't clear flag if DIAG is still up
    this->diag_handler_.check(this->diag_triggered_);
    this->poll_driver_status_();
  }
#endif
}

bool TMC2209Component::is_stalled() {
  const auto sgthrs = this->read_register(SGTHRS);
  const auto sgresult = this->read_register(SG_RESULT);
  return (2 * sgthrs) > sgresult;
}

uint16_t TMC2209Component::get_microsteps() { return MRES_TO_MS(this->read_field(MRES_FIELD)); }
void TMC2209Component::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 0; mres <= 8; mres++) {
    if (MRES_TO_MS(mres) == ms) {
      return this->write_field(MRES_FIELD, mres);
    }
  }

  ESP_LOGW(TAG, "%d is not a valid microstepping option", ms);
}

float TMC2209Component::get_motor_load() {
  const uint16_t result = this->read_register(SG_RESULT);
  return (510.0 - (float) result) / (510.0 - this->read_register(SGTHRS) * 2.0);
}

float TMC2209Component::read_vsense() { return (this->read_field(VSENSE_FIELD) ? VSENSE_LOW : VSENSE_HIGH); }

uint16_t TMC2209Component::current_scale_to_rms_current_mA(uint8_t cs) {
  cs = clamp<uint8_t>(cs, 0, 31);
  if (cs == 0)
    return 0;
  return (cs + 1) / 32.0f * this->read_vsense() / (this->rsense_ + 0.02f) * (1 / sqrtf(2)) * 1000.0f;
}

uint8_t TMC2209Component::rms_current_to_current_scale_mA_no_clamp(uint16_t mA) {
  return 32.0f * sqrtf(2) * FROM_MILLI(mA) * ((this->rsense_ + 0.02f) / this->read_vsense()) - 1.0f;
}

uint8_t TMC2209Component::rms_current_to_current_scale_mA(uint16_t mA) {
  if (mA == 0) {
    return 0;
  }
  const uint8_t cs = this->rms_current_to_current_scale_mA_no_clamp(mA);

  if (cs > 31) {
    const uint16_t mA_limit = this->current_scale_to_rms_current_mA(cs);
    ESP_LOGW(TAG, "Configured RSense and VSense has a max current limit of %d mA. Clamping value to max!", mA_limit);
  }

  return clamp<uint8_t>(cs, 0, 31);
}

void TMC2209Component::write_run_current_mA(uint16_t mA) {
  const uint8_t cs = this->rms_current_to_current_scale_mA(mA);
  this->write_field(IRUN_FIELD, cs);
}

uint16_t TMC2209Component::read_run_current_mA() {
  const uint8_t cs = this->read_field(IRUN_FIELD);
  return this->current_scale_to_rms_current_mA(cs);
}

void TMC2209Component::write_hold_current_mA(uint16_t mA) {
  const uint8_t cs = this->rms_current_to_current_scale_mA(mA);
  this->write_field(IHOLD_FIELD, cs);
}

uint16_t TMC2209Component::read_hold_current_mA() {
  const uint8_t cs = this->read_field(IHOLD_FIELD);
  return this->current_scale_to_rms_current_mA(cs);
}

void TMC2209Component::set_tpowerdown_ms(uint32_t delay_in_ms) {
  auto tpowerdown = ((float) delay_in_ms / 262144.0) * ((float) this->clk_frequency_ / 1000.0);
  this->write_field(TPOWERDOWN_FIELD, tpowerdown);
};

uint32_t TMC2209Component::get_tpowerdown_ms() {
  return (this->read_field(TPOWERDOWN_FIELD) * 262144.0) / (this->clk_frequency_ / 1000.0);
};

std::tuple<uint8_t, uint8_t> TMC2209Component::unpack_ottrim_values(uint8_t ottrim) {
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

void TMC2209Component::poll_driver_status_() {
  const uint32_t drv_status = this->read_register(DRV_STATUS);
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
  this->uvcp_handler_.check(this->read_field(UV_CP_FIELD));
}

}  // namespace tmc2209
}  // namespace esphome
