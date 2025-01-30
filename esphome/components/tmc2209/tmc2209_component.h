#pragma once

#include "tmc2209_api_registers.h"
#include "tmc2209_api.h"
#include "events.h"
#include "tmc2209_config_dumps.h"

#include "esphome/core/component.h"
#include "esphome/core/optional.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tmc2209 {

#define FROM_MILLI(mu) ((double) mu / 1000.0)
#define TO_MILLI(u) (u * 1000.0)

#define MRES_TO_MS(mres) (256 >> mres)  // convert MRES value (microstepping index) to human readable microstep value

enum CurrentScaleMode : bool {
  VREF = true,
  REGISTER = false,
};

enum StandstillMode : uint8_t {
  NORMAL = 0,
  FREEWHEELING = 1,
  COIL_SHORT_LS = 3,
  COIL_SHORT_HS = 4,
};

enum ShaftDirection : uint8_t {
  CLOCKWISE = 0,
  COUNTERCLOCKWISE = 1,
};

struct ISRPinTriggerStore {
  bool *pin_triggered_ptr = nullptr;
  static void IRAM_ATTR HOT pin_isr(ISRPinTriggerStore *arg) { (*(arg->pin_triggered_ptr)) = true; }
};

class TMC2209Component : public TMC2209API, public Component {
 public:
  TMC2209Component() = default;
  TMC2209Component(uint8_t address) : TMC2209API(address){};

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;

  // Component setters
  void set_enn_pin(InternalGPIOPin *pin) { this->enn_pin_ = pin; };
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; };
  void set_index_pin(InternalGPIOPin *pin) { this->index_pin_ = pin; };
  void set_step_pin(GPIOPin *pin) { this->step_pin_ = pin; };
  void set_dir_pin(GPIOPin *pin) { this->dir_pin_ = pin; };

  void set_clk_freq(uint32_t clk_freq) {
    this->clk_freq_ = clk_freq;
    this->clk_to_vactual_factor_ = ((float) clk_freq / 16777216);
  }

  void set_rsense(float rsense) { this->rsense_ = rsense; }
  void set_analog_current_scale(bool enable) { this->use_analog_current_scale_ = enable; }
  void set_vsense(bool vsense) { this->vsense_ = vsense; }
  void set_ottrim(uint8_t ottrim) { this->ottrim_ = ottrim; }
  void set_enable_driver_health_check(bool enable) { this->driver_health_check_is_enabled_ = enable; }
  void set_enable_stall_detection(bool enable) { this->stall_detection_is_enabled_ = enable; }
  void set_config_dump_include_registers(bool include) { this->config_dump_include_registers_ = include; }
  void set_toff_recovery(bool enable) { this->toff_recovery_ = enable; }
  void add_on_stall_callback(std::function<void()> &&callback) { this->on_stall_callback_.add(std::move(callback)); }
  void add_on_driver_status_callback(std::function<void(DriverStatusEvent)> &&callback) {
    this->on_driver_status_callback_.add(std::move(callback));
  }

  // virtual void enable(bool enable, bool recover_toff = true);
  virtual void enable(bool enable);

  float read_vsense();
  void set_microsteps(uint16_t ms);
  uint16_t get_microsteps();
  float get_motor_load();
  virtual bool is_stalled();
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

  // clock compensated velocity (VACTUAL)
  int32_t vactual_to_speed(int32_t vactual) { return std::round((float) vactual * this->clk_to_vactual_factor_); }
  int32_t speed_to_vactual(int32_t speed) { return std::round((float) speed / this->clk_to_vactual_factor_); }
  int32_t read_speed() { return this->vactual_to_speed((int32_t) this->read_field(VACTUAL_FIELD)); }
  void write_speed(int32_t speed) { this->write_field(VACTUAL_FIELD, this->speed_to_vactual(speed)); }

 protected:
  /** Setup / configuration */
  bool use_analog_current_scale_{false};
  bool config_dump_include_registers_{false};
  optional<float> rsense_{};
  optional<bool> vsense_{};
  optional<uint8_t> ottrim_{};
  uint32_t clk_freq_;
  float clk_to_vactual_factor_{0.715};  // coefficient between clock and real steps using VACTUAL
  bool toff_recovery_{true};

  bool driver_health_check_is_enabled_{false};
  bool stall_detection_is_enabled_{false};

  InternalGPIOPin *enn_pin_{nullptr};
  InternalGPIOPin *diag_pin_{nullptr};
  InternalGPIOPin *index_pin_{nullptr};
  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  /** */

  /** Runtime helpers */
  bool is_enabled_{false};
  bool check_gstat_{false};
  bool check_drv_status{false};
  bool diag_triggered_{false};
  optional<uint8_t> toff_storage_{};
  /** */

  ISRPinTriggerStore diag_isr_store_;
  HighFrequencyLoopRequester high_freq_;

  CallbackManager<void()> on_stall_callback_;
  CallbackManager<void(const DriverStatusEvent &event)> on_driver_status_callback_;

  EventHandler stall_handler_;
  EventHandler reset_handler_;
  EventHandler drv_err_handler_;
  EventHandler diag_handler_;   // Event on DIAG
  EventHandler otpw_handler_;   // overtemperature prewarning flag (Selected limit has been reached)
  EventHandler ot_handler_;     // overtemperature flag (Selected limit has been reached)
  EventHandler t120_handler_;   // 120°C comparator (Temperature threshold is exceeded)
  EventHandler t143_handler_;   // 143°C comparator (Temperature threshold is exceeded)
  EventHandler t150_handler_;   // 150°C comparator (Temperature threshold is exceeded)
  EventHandler t157_handler_;   // 157°C comparator (Temperature threshold is exceeded)
  EventHandler olb_handler_;    // open load indicator phase B
  EventHandler ola_handler_;    // open load indicator phase B
  EventHandler s2vsb_handler_;  // low side short indicator phase B
  EventHandler s2vsa_handler_;  // low side short indicator phase A
  EventHandler s2gb_handler_;   // short to ground indicator phase B
  EventHandler s2ga_handler_;   // short to ground indicator phase A
  EventHandler uvcp_handler_;   // Charge pump undervoltage
};

}  // namespace tmc2209
}  // namespace esphome
