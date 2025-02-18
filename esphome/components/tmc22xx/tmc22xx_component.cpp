#include "tmc22xx_registers.h"
#include "tmc22xx_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc22xx {

void TMC22XXComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TMC22XX Component...");

  this->high_freq_.start();

  if (this->enn_pin_ != nullptr) {
    this->enn_pin_->setup();
  }

  if (this->index_pin_ != nullptr) {
    this->index_pin_->setup();
  }

  if (this->step_pin_ != nullptr and this->dir_pin_ != nullptr) {
    this->step_pin_->setup();
    this->dir_pin_->setup();
  }

  if (this->diag_pin_ != nullptr) {
    this->diag_pin_->setup();
    this->diag_pin_->attach_interrupt(ISRPinTriggerStore::pin_isr, &this->diag_isr_store_, gpio::INTERRUPT_RISING_EDGE);
    this->diag_isr_store_.pin_triggered_ptr = &this->diag_triggered_;
  }

  if (!this->read_field(VERSION_FIELD)) {
    this->status_set_error("Failed to communicate with driver");
    this->mark_failed();
  }

  this->write_field(PDN_DISABLE_FIELD, true);
  this->write_field(TEST_MODE_FIELD, false);
  this->write_field(SHAFT_FIELD, false);
  this->write_field(MSTEP_REG_SELECT_FIELD, true);
  this->write_field(INTERNAL_RSENSE_FIELD, !this->rsense_.has_value());
  this->write_field(I_SCALE_ANALOG_FIELD, this->use_analog_current_scale_);

  if (this->vsense_.has_value()) {
    this->write_field(VSENSE_FIELD, this->vsense_.value());
  }

  if (this->ottrim_.has_value()) {
    this->write_field(OTTRIM_FIELD, this->ottrim_.value());
  }

  if (this->toff_recovery_ && !this->toff_storage_.has_value()) {
    const uint8_t toff_ = this->read_field(TOFF_FIELD);
    if (toff_ == 0) {
      ESP_LOGW(TAG, "captured TOFF value was 0 and will not be used for recovery (as it will disable the driver)");
    } else {
      this->toff_storage_ = toff_;
    }
  }

  this->diag_handler_.set_callbacks(  // DIAG
      [this]() {
        ESP_LOGV(TAG, "Executing DIAG rise event");
        // TODO: Handle Power-on reset ??
        const int32_t gstat = this->read_register(GSTAT);
        this->check_gstat_ = (bool) gstat;

        // this->stall_handler_.check(gstat == 0b000);
        if (gstat == 0b000) {
          this->on_stall_callback_.call();
        }

        this->reset_handler_.check((bool) this->extract_field(gstat, RESET_FIELD));
        this->drv_err_handler_.check((bool) this->extract_field(gstat, DRV_ERR_FIELD));
        this->uvcp_handler_.check((bool) this->extract_field(gstat, UV_CP_FIELD));

        this->on_driver_status_callback_.call(DIAG_TRIGGERED);
      },  // rise
      [this]() {
        ESP_LOGV(TAG, "Executing DIAG fall event");
        const int32_t gstat = this->read_register(GSTAT);
        this->check_gstat_ = (bool) gstat;
        this->reset_handler_.check((bool) this->extract_field(gstat, RESET_FIELD));
        this->drv_err_handler_.check((bool) this->extract_field(gstat, DRV_ERR_FIELD));
        this->uvcp_handler_.check((bool) this->extract_field(gstat, UV_CP_FIELD));

        this->on_driver_status_callback_.call(DIAG_TRIGGER_CLEARED);
      }  // fall
  );

  this->stall_handler_.set_on_rise_callback([this]() { this->on_stall_callback_.call(); });

  this->reset_handler_.set_callbacks(  // gstat reset
      [this]() {                       // rise
        this->write_field(RESET_FIELD, 1);
        this->on_driver_status_callback_.call(RESET);
      },
      [this]() {  // fall
        this->write_field(RESET_FIELD, 1);
        this->on_driver_status_callback_.call(RESET_CLEARED);
      });

  this->drv_err_handler_.set_callbacks(  // gstat drv_err
      [this]() {                         // rise
        this->write_field(DRV_ERR_FIELD, 1);

        const int32_t drv_status = this->read_register(DRV_STATUS);
        this->ot_handler_.check((bool) this->extract_field(drv_status, OT_FIELD));
        this->otpw_handler_.check((bool) this->extract_field(drv_status, OTPW_FIELD));
        this->t157_handler_.check((bool) this->extract_field(drv_status, T157_FIELD));
        this->t150_handler_.check((bool) this->extract_field(drv_status, T150_FIELD));
        this->t143_handler_.check((bool) this->extract_field(drv_status, T143_FIELD));
        this->t120_handler_.check((bool) this->extract_field(drv_status, T120_FIELD));
        this->ola_handler_.check((bool) this->extract_field(drv_status, OLA_FIELD));
        this->olb_handler_.check((bool) this->extract_field(drv_status, OLB_FIELD));
        this->s2vsa_handler_.check((bool) this->extract_field(drv_status, S2VSA_FIELD));
        this->s2vsb_handler_.check((bool) this->extract_field(drv_status, S2VSB_FIELD));
        this->s2ga_handler_.check((bool) this->extract_field(drv_status, S2GA_FIELD));
        this->s2gb_handler_.check((bool) this->extract_field(drv_status, S2GB_FIELD));
        this->on_driver_status_callback_.call(DRIVER_ERROR);
      },
      [this]() {  // fall
        ESP_LOGV(TAG, "Executing driver err fall event");
        this->on_driver_status_callback_.call(DRIVER_ERROR_CLEARED);
      });

  this->uvcp_handler_.set_callbacks(  // gstat uc_vp
      [this]() {                      // rise
        this->write_field(UV_CP_FIELD, 1);
        this->on_driver_status_callback_.call(CP_UNDERVOLTAGE);
      },
      [this]() {  // fall
        ESP_LOGV(TAG, "Executing GSTAT UCVP fall event");
        this->on_driver_status_callback_.call(CP_UNDERVOLTAGE_CLEARED);
      });

  this->otpw_handler_.set_callbacks(  // drv_status overtemperature prewarning
      [this]() { this->on_driver_status_callback_.call(OVERTEMPERATURE_PREWARNING); },
      [this]() { this->on_driver_status_callback_.call(OVERTEMPERATURE_PREWARNING_CLEARED); });

  this->ot_handler_.set_callbacks(  // drv_status overtemperature
      [this]() { this->on_driver_status_callback_.call(OVERTEMPERATURE); },
      [this]() { this->on_driver_status_callback_.call(OVERTEMPERATURE_CLEARED); });

  this->t120_handler_.set_callbacks(                                                // drv_status t120 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_120C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_120C); }   // fall
  );

  this->t143_handler_.set_callbacks(                                                // drv_status t143 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_143C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_143C); }   // fall
  );

  this->t150_handler_.set_callbacks(                                                // drv_status t150 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_150C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_150C); }   // fall
  );

  this->t157_handler_.set_callbacks(                                                // drv_status t157 flag
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_ABOVE_157C); },  // rise
      [this]() { this->on_driver_status_callback_.call(TEMPERATURE_BELOW_157C); }   // fall
  );

  this->ola_handler_.set_callbacks(  // drv_status ola
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD);
        this->on_driver_status_callback_.call(OPEN_LOAD_A);
      },
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD_CLEARED);
        this->on_driver_status_callback_.call(OPEN_LOAD_A_CLEARED);
      });

  this->olb_handler_.set_callbacks(  // drv_status olb
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD);
        this->on_driver_status_callback_.call(OPEN_LOAD_B);
      },
      [this]() {
        this->on_driver_status_callback_.call(OPEN_LOAD_CLEARED);
        this->on_driver_status_callback_.call(OPEN_LOAD_B_CLEARED);
      });

  this->s2vsa_handler_.set_callbacks(  // drv_status s2vsa
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_A);
      },
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_CLEARED);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_A_CLEARED);
      });

  this->s2vsb_handler_.set_callbacks(  // drv_status s2vsb
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_B);
      },
      [this]() {
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_CLEARED);
        this->on_driver_status_callback_.call(LOW_SIDE_SHORT_B_CLEARED);
      });

  this->s2ga_handler_.set_callbacks(  // drv_status s2ga
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT);
        this->on_driver_status_callback_.call(GROUND_SHORT_A);
      },
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT_CLEARED);
        this->on_driver_status_callback_.call(GROUND_SHORT_A_CLEARED);
      });

  this->s2gb_handler_.set_callbacks(  // drv_status s2gb
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT);
        this->on_driver_status_callback_.call(GROUND_SHORT_B);
      },
      [this]() {
        this->on_driver_status_callback_.call(GROUND_SHORT_CLEARED);
        this->on_driver_status_callback_.call(GROUND_SHORT_B_CLEARED);
      });

  ESP_LOGCONFIG(TAG, "TMC22XX Component setup done.");
}

void TMC22XXComponent::loop() {
  if (this->driver_health_check_is_enabled_ or this->stall_detection_is_enabled_) {
    if (this->diag_pin_ != nullptr) {
      this->diag_handler_.check(this->diag_triggered_);
      if (this->diag_triggered_) {
        this->diag_triggered_ = this->diag_pin_->digital_read();  // don't clear flag if DIAG is still up
      }
    } else {
      const int32_t ioin = this->read_register(IOIN);
      this->diag_handler_.check((bool) this->extract_field(ioin, DIAG_FIELD));
      // TODO: maybe do something with INDEX for warnings
    }
  }
}

bool TMC22XXComponent::is_stalled() {
  if ((bool) this->read_field(STST_FIELD)) {
    return false;
  }

  const int32_t sgthrs = this->read_register(SGTHRS);
  const int32_t sgresult = this->read_register(SG_RESULT);
  return (sgthrs << 1) > sgresult;
}

uint16_t TMC22XXComponent::get_microsteps() { return MRES_TO_MS(this->read_field(MRES_FIELD)); }
void TMC22XXComponent::set_microsteps(uint16_t ms) {
  for (uint8_t mres = 0; mres <= 8; mres++) {
    if (MRES_TO_MS(mres) == ms) {
      return this->write_field(MRES_FIELD, mres);
    }
  }

  ESP_LOGW(TAG, "%d is not a valid microstepping option", ms);
}

float TMC22XXComponent::get_motor_load() {
  const int32_t result = this->read_register(SG_RESULT);
  return (510.0 - (float) result) / (510.0 - (int32_t) this->read_register(SGTHRS) * 2.0);
}

float TMC22XXComponent::read_vsense() { return (this->read_field(VSENSE_FIELD) ? 0.180f : 0.325f); }

uint16_t TMC22XXComponent::current_scale_to_rms_current_mA(uint8_t cs) {
  cs = clamp<uint8_t>(cs, 0, 31);
  if (cs == 0)
    return 0;
  return (cs + 1) / 32.0f * this->read_vsense() / (this->rsense_.value_or(0.170) + 0.02f) * (1 / sqrtf(2)) * 1000.0f;
}

uint8_t TMC22XXComponent::rms_current_to_current_scale_mA_no_clamp(uint16_t mA) {
  return 32.0f * sqrtf(2) * FROM_MILLI(mA) * ((this->rsense_.value_or(0.170) + 0.02f) / this->read_vsense()) - 1.0f;
}

uint8_t TMC22XXComponent::rms_current_to_current_scale_mA(uint16_t mA) {
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

void TMC22XXComponent::write_run_current_mA(uint16_t mA) {
  const uint8_t cs = this->rms_current_to_current_scale_mA(mA);
  this->write_field(IRUN_FIELD, cs);
}

uint16_t TMC22XXComponent::read_run_current_mA() {
  const uint8_t cs = this->read_field(IRUN_FIELD);
  return this->current_scale_to_rms_current_mA(cs);
}

void TMC22XXComponent::write_hold_current_mA(uint16_t mA) {
  const uint8_t cs = this->rms_current_to_current_scale_mA(mA);
  this->write_field(IHOLD_FIELD, cs);
}

uint16_t TMC22XXComponent::read_hold_current_mA() {
  const uint8_t cs = this->read_field(IHOLD_FIELD);
  return this->current_scale_to_rms_current_mA(cs);
}

void TMC22XXComponent::set_tpowerdown_ms(uint32_t delay_in_ms) {
  auto tpowerdown = ((float) delay_in_ms / 262144.0) * ((float) this->clk_freq_ / 1000.0);
  this->write_field(TPOWERDOWN_FIELD, tpowerdown);
};

uint32_t TMC22XXComponent::get_tpowerdown_ms() {
  return (this->read_field(TPOWERDOWN_FIELD) * 262144.0) / (this->clk_freq_ / 1000.0);
};

std::tuple<uint8_t, uint8_t> TMC22XXComponent::unpack_ottrim_values(uint8_t ottrim) {
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

// void TMC22XXComponent::enable(bool enable, bool recover_toff = true) {
void TMC22XXComponent::enable(bool enable) {
  if (this->enn_pin_ != nullptr) {
    // Use ENN pin to handle enable/disable
    this->enn_pin_->digital_write(!enable);
  } else {
    // Use UART to handle enable/disable
    if (enable) {
      if (this->toff_recovery_) {
        // TODO: maybe handle TBL < 2 and TOFF = [1;2]
        this->write_field(TOFF_FIELD, this->toff_storage_.value_or(3));
      }
    } else {
      if (this->toff_recovery_ && (enable != this->is_enabled_)) {
        // don't capture current toff as it could be 0 due to the driver being disabled
        this->toff_storage_ = this->read_field(TOFF_FIELD);
      }
      this->write_field(TOFF_FIELD, 0);
    }
  }

  this->is_enabled_ = enable;
}

}  // namespace tmc22xx
}  // namespace esphome
