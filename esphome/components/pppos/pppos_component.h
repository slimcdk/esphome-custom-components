#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"

#include "netif/ppp/pppos.h"
#include "esp_netif.h"
#include "esp_event.h"

#include <string.h>

namespace esphome {
namespace ppp {

enum pppos_state {
  PPPOS_IDLE = 0,
  PPPOS_INIT = 1,
  PPPOS_START_SEQ = 12,
  PPPOS_RESET = 2,
  PPPOS_RESET_HOLD = 3,
  PPPOS_RESET_WAIT = 4,
  PPPOS_BEGIN_WAIT = 5,
  PPPOS_BEGIN = 6,
  PPPOS_SETUP = 7,
  PPPOS_CMD = 8,
  PPPOS_CMD_RESP = 9,
  PPPOS_START_PPP = 10,
  PPPOS_RUN = 11,
  PPPOS_CLOSING = 13,
};

enum class PPPoSComponentState {
  STOPPED,
  CONNECTING,
  CONNECTED,
};

struct ManualIP {
  network::IPAddress static_ip;
  network::IPAddress gateway;
  network::IPAddress subnet;
  network::IPAddress dns1;  ///< The first DNS server. 0.0.0.0 for default.
  network::IPAddress dns2;  ///< The second DNS server. 0.0.0.0 for default.
};

class PPPoSComponent : public Component, public uart::UARTDevice {
 public:
  PPPoSComponent();
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  bool can_proceed() override;
  bool is_connected();

  void set_manual_ip(const ManualIP &manual_ip);

  network::IPAddress get_ip_address();
  std::string get_use_address() const;
  void set_use_address(const std::string &use_address);

 protected:
  static void status_cb(ppp_pcb *pcb, int err_code, void *ctx);
  static uint32_t output_cb(ppp_pcb *pcb, uint8_t *data, uint32_t len, void *ctx);

  optional<ManualIP> manual_ip_{};
  std::string use_address_;

  PPPoSComponentState state_{PPPoSComponentState::STOPPED};
  ppp_pcb *ppp_control_block_ = nullptr;
  struct netif ppp_netif_;
  // esp_netif_t *ppp_netif_{nullptr};

  void set_state(PPPoSComponentState new_state);

  // bool started_{false};
  // bool connected_{false};
  // uint32_t connect_begin_;

 private:
  std::string state_to_string(PPPoSComponentState state);
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern PPPoSComponent *global_pppos_component;

}  // namespace ppp
}  // namespace esphome
