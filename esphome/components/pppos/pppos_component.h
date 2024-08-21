#pragma once

#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"

#include "netif/ppp/pppos.h"
// #include "netif/ppp/pppapi.h"
#include <lwip/dns.h>
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

struct NetworkDNS {
  network::IPAddress dns1;  ///< The first DNS server. 0.0.0.0 for default.
  network::IPAddress dns2;  ///< The second DNS server. 0.0.0.0 for default.
};

class PPPoSComponent : public Component, public uart::UARTDevice {
 public:
  PPPoSComponent();

  void set_network_dns(const NetworkDNS &network_dns) { this->network_dns_ = network_dns; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const { return setup_priority::WIFI; }  // TODO: Add PPP in core

  bool can_proceed() override { return this->is_connected() || this->is_failed(); }
  bool is_connected() { return this->state_ == PPPoSComponentState::CONNECTED; }
  network::IPAddress get_ip_address() { return network::IPAddress(this->ppp_netif_.ip_addr.addr); }
  std::string get_use_address() const { return {}; };  // TODO: Will an empty address cause issues ?

 protected:
  static void status_cb(ppp_pcb *pcb, int err_code, void *ctx);
  static uint32_t output_cb(ppp_pcb *pcb, uint8_t *data, uint32_t len, void *ctx);
  // static void netif_status_callback(struct netif *ppp_netif_);
  void set_state(PPPoSComponentState new_state);

  PPPoSComponentState state_{PPPoSComponentState::STOPPED};
  struct netif ppp_netif_;
  esp_netif_t *_ppp_netif_{nullptr};

  ppp_pcb *ppp_control_block_ = nullptr;
  optional<NetworkDNS> network_dns_{};

 private:
  std::string state_to_string(PPPoSComponentState state);
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern PPPoSComponent *global_pppos_component;

}  // namespace ppp
}  // namespace esphome
