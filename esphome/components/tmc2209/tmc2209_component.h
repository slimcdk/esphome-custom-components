#pragma once

#include "events.h"
#include "tmc2209_api.h"
#include "tmc2209_api_registers.h"

namespace esphome {
namespace tmc2209 {

#define FROM_MILLI(mu) (mu / 1000.0)
#define TO_MILLI(u) (u * 1000)

#define VSENSE_HIGH 0.325f
#define VSENSE_LOW 0.180f

struct ISRPinTriggerStore {
  bool *pin_triggered_ptr{nullptr};
  static void IRAM_ATTR HOT pin_isr(ISRPinTriggerStore *arg) { (*(arg->pin_triggered_ptr)) = true; }
};

class TMC2209Component : public TMC2209API, public Component {
 public:
  TMC2209Component(uint8_t address, uint32_t clk_frequency, bool internal_sense, float rsense)
      : TMC2209API(address),
        clk_frequency_(clk_frequency),
        internal_sense_(internal_sense),
        rsense_(rsense)  // Initialize rsense_ first
        {};

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;

  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; };
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; };
  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; };
  void set_dir_pin(GPIOPin *pin) { this->dir_pin_ = pin; };

  float read_vsense();

  uint8_t mres_to_microsteps(uint8_t mres);
  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();
  float get_motor_load();
  bool is_stalled();
  void set_tpowerdown_ms(uint32_t delay_in_ms);
  uint32_t get_tpowerdown_ms();
  std::tuple<uint8_t, uint8_t> unpack_ottrim_values(uint8_t ottrim);

  // Read, write and convert run/hold currents
  uint8_t rms_current_to_current_scale_mA_no_clamp(uint16_t mA);
  uint16_t current_scale_to_rms_current_mA(uint8_t cs);
  uint8_t rms_current_to_current_scale_mA(uint16_t mA);
  void write_run_current_mA(uint16_t mA);
  void write_hold_current_mA(uint16_t mA);
  uint16_t read_run_current_mA();
  uint16_t read_hold_current_mA();
  void write_run_current(float A) { this->write_run_current_mA(TO_MILLI(A)); };
  void write_hold_current(float A) { this->write_hold_current_mA(TO_MILLI(A)); };
  float read_run_current() { return FROM_MILLI(this->read_run_current_mA()); };
  float read_hold_current() { return FROM_MILLI(this->read_hold_current_mA()); };

  void set_stall_detection_activation_level(float level) { this->sdal_ = level; };

  void add_on_event_callback(std::function<void(DriverEvent)> &&callback) {
    this->on_event_callback_.add(std::move(callback));
  }

 protected:
  const uint32_t clk_frequency_;
  const bool internal_sense_;
  const float rsense_;  // default RDSon value

  InternalGPIOPin *enn_pin_{nullptr};
  InternalGPIOPin *diag_pin_{nullptr};
  InternalGPIOPin *index_pin_{nullptr};
  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};

  bool is_enabled_;
  float sdal_;  // stall detection activation level. A percentage of max_speed.

  void check_driver_status_();

  CallbackManager<void(const DriverEvent &event)> on_event_callback_;
  EventHandler diag_handler_{};         // Event on DIAG
  EventHandler stalled_handler_{true};  // Stalled. Initial state is true so that first startup doesn't trigger
  // EventHandler stst_handler_{};     // standstill indicator
  // EventHandler stealth_handler_{};  // StealthChop indicator (0=SpreadCycle mode, 1=StealthChop mode)
  EventHandler otpw_handler_{};   // overtemperature prewarning flag (Selected limit has been reached)
  EventHandler ot_handler_{};     // overtemperature flag (Selected limit has been reached)
  EventHandler t120_handler_{};   // 120°C comparator (Temperature threshold is exceeded)
  EventHandler t143_handler_{};   // 143°C comparator (Temperature threshold is exceeded)
  EventHandler t150_handler_{};   // 150°C comparator (Temperature threshold is exceeded)
  EventHandler t157_handler_{};   // 157°C comparator (Temperature threshold is exceeded)
  EventHandler olb_handler_{};    // open load indicator phase B
  EventHandler ola_handler_{};    // open load indicator phase B
  EventHandler s2vsb_handler_{};  // low side short indicator phase B
  EventHandler s2vsa_handler_{};  // low side short indicator phase A
  EventHandler s2gb_handler_{};   // short to ground indicator phase B
  EventHandler s2ga_handler_{};   // short to ground indicator phase A
  EventHandler uvcp_handler_{};   // Charge pump undervoltage

  ISRPinTriggerStore diag_isr_store_{};
  ISRPinTriggerStore index_isr_store_{};

  bool diag_triggered_{false};
  bool index_triggered_{false};

  HighFrequencyLoopRequester high_freq_;
};

#define LOG_TMC2209_IC_VERSION(this) \
  const int8_t icv_ = this->read_field(VERSION_FIELD); \
  if (std::isnan(icv_) || icv_ == 0) { \
    ESP_LOGE(TAG, "  Unable to read IC version. Is the driver powered and wired correctly?"); \
  } else if (icv_ == IC_VERSION_33) { \
    ESP_LOGCONFIG(TAG, "  Detected IC version: 0x%02X", icv_); \
  } else { \
    ESP_LOGE(TAG, "  Detected unknown IC version: 0x%02X", icv_); \
  }

#define LOG_TMC2209_PINS(this) \
  if (this->enn_pin_) { \
    LOG_PIN("  ENN Pin: ", this->enn_pin_); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Enable/disable driver with TOFF"); \
  } \
  if (this->diag_pin_) { \
    LOG_PIN("  DIAG Pin: ", this->diag_pin_); \
  } else { \
    ESP_LOGCONFIG(TAG, "  Driver status poll interval: %dms", POLL_STATUS_INTERVAL); \
  } \
  LOG_PIN("  INDEX Pin: ", this->index_pin_); \
  LOG_PIN("  STEP Pin: ", this->step_pin_); \
  LOG_PIN("  DIR Pin: ", this->dir_pin_);

#define LOG_TMC2209_CURRENTS(this) \
  ESP_LOGCONFIG(TAG, "  Currents:"); \
  if (this->read_field(INTERNAL_RSENSE_FIELD)) { \
    ESP_LOGCONFIG(TAG, "    RSense: %.3f Ohm (internal RDSon value)", this->rsense_); \
  } else { \
    ESP_LOGCONFIG(TAG, "    RSense: %.3f Ohm (external sense resistors)", this->rsense_); \
    if (this->read_field(VSENSE_FIELD)) { \
      ESP_LOGCONFIG(TAG, "    VSense: True (low heat dissipation)"); \
    } else { \
      ESP_LOGCONFIG(TAG, "    VSense: False (high heat dissipation)"); \
    } \
  } \
  ESP_LOGCONFIG(TAG, "    Currently set IRUN: %d (%d mA)", this->read_field(IRUN_FIELD), this->read_run_current_mA()); \
  ESP_LOGCONFIG(TAG, "    Currently set IHOLD: %d (%d mA)", this->read_field(IHOLD_FIELD), \
                this->read_hold_current_mA()); \
  ESP_LOGCONFIG(TAG, "    Limits: %d mA", this->current_scale_to_rms_current_mA(31));

#define LOG_TMC2209_REGISTER_DUMP(this) \
  ESP_LOGCONFIG(TAG, "  Register dump:"); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "GCONF:", this->read_register(GCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "GSTAT:", this->read_register(GSTAT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IFCNT:", this->read_register(IFCNT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SLAVECONF:", this->read_register(SLAVECONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "OTP_PROG:", this->read_register(OTP_PROG)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "OTP_READ:", this->read_register(OTP_READ)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IOIN:", this->read_register(IOIN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "FACTORY_CONF:", this->read_register(FACTORY_CONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "IHOLD_IRUN:", this->read_register(IHOLD_IRUN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TPOWERDOWN:", this->read_register(TPOWERDOWN)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TSTEP:", this->read_register(TSTEP)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TPWMTHRS:", this->read_register(TPWMTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "TCOOLTHRS:", this->read_register(TCOOLTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "VACTUAL:", this->read_register(VACTUAL)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SGTHRS:", this->read_register(SGTHRS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "SG_RESULT:", this->read_register(SG_RESULT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "COOLCONF:", this->read_register(COOLCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "MSCNT:", this->read_register(MSCNT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "MSCURACT:", this->read_register(MSCURACT)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "CHOPCONF:", this->read_register(CHOPCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "DRV_STATUS:", this->read_register(DRV_STATUS)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWMCONF:", this->read_register(PWMCONF)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWMSCALE:", this->read_register(PWMSCALE)); \
  ESP_LOGCONFIG(TAG, "    %-13s 0x%08X", "PWM_AUTO:", this->read_register(PWM_AUTO));

#define LOG_TMC2209(this) \
  LOG_TMC2209_PINS(this); \
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->driver_address_); \
  LOG_TMC2209_IC_VERSION(this); \
  ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->get_microsteps()); \
  ESP_LOGCONFIG(TAG, "  Clock frequency: %d Hz", this->clk_frequency_); \
  const auto [otpw, ot] = this->unpack_ottrim_values(this->read_field(OTTRIM_FIELD)); \
  ESP_LOGCONFIG(TAG, "  Overtemperature: prewarning = %dC | shutdown = %dC", otpw, ot); \
  LOG_TMC2209_CURRENTS(this); \
  LOG_TMC2209_REGISTER_DUMP(this);

}  // namespace tmc2209
}  // namespace esphome
