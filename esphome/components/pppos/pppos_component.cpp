/** https://www.nongnu.org/lwip/2_0_x/group__ppp.html **/

#include "pppos_component.h"

namespace esphome {
namespace ppp {

#define ESPHL_ERROR_CHECK(err, message) \
  if ((err) != ESP_OK) { \
    ESP_LOGE(TAG, message ": (%d) %s", err, esp_err_to_name(err)); \
    this->mark_failed(); \
    return; \
  }

static const char *TAG = "pppos";

PPPoSComponent *global_pppos_component;
PPPoSComponent::PPPoSComponent() { global_pppos_component = this; }

// void PPPoSComponent::netif_status_callback(struct netif *nif) {
//   ESP_LOGV(TAG, "PPPNETIF: %c%c%d is %s", nif->name[0], nif->name[1], nif->num, netif_is_up(nif) ? "UP" : "DOWN");
//   ESP_LOGV(TAG, "IPV4: Host at %s ", ip4addr_ntoa(netif_ip4_addr(nif)));
//   ESP_LOGV(TAG, "mask %s ", ip4addr_ntoa(netif_ip4_netmask(nif)));
//   ESP_LOGV(TAG, "gateway %s", ip4addr_ntoa(netif_ip4_gw(nif)));
// }

void PPPoSComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PPPoS...");

  ESPHL_ERROR_CHECK(esp_netif_init(), "netif init error");
  ESPHL_ERROR_CHECK(esp_event_loop_create_default(), "event loop error");

  // esp_netif_config_t cfg = ESP_NETIF_DEFAULT_PPP();
  // this->_ppp_netif_ = esp_netif_new(&cfg);

  this->ppp_control_block_ =
      pppos_create(&this->ppp_netif_, global_pppos_component->output_cb, global_pppos_component->status_cb, this);
  if (this->ppp_control_block_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create PPP interface");
    this->mark_failed();
    return;
  }

  // ESPHL_ERROR_CHECK(esp_netif_attach(this->_ppp_netif_, esp_ppp_new_netif_glue(&this->ppp_netif_)),
  //                   "error attaching ppp interface");

  ppp_set_default(this->ppp_control_block_);

#if LWIP_DNS
  if (uint32_t(this->network_dns_->dns1) != 0) {
    ip_addr_t d;
    d.addr = static_cast<uint32_t>(this->network_dns_->dns1);
    dns_setserver(0, &d);
  }
  if (uint32_t(this->network_dns_->dns2) != 0) {
    ip_addr_t d;
    d.addr = static_cast<uint32_t>(this->network_dns_->dns2);
    dns_setserver(1, &d);
  }
#endif
  ppp_set_usepeerdns(this->ppp_control_block_, 1);

  if (ppp_connect(this->ppp_control_block_, 0) != ERR_OK) {
    ESP_LOGE(TAG, "Couldnt initiate connection");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Setup done...");
  this->set_state(PPPoSComponentState::CONNECTING);
}

void PPPoSComponent::status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
  LWIP_UNUSED_ARG(ctx);

  struct netif *pppif = ppp_netif(pcb);

  switch (err_code) {
    case PPPERR_NONE:
      global_pppos_component->set_state(PPPoSComponentState::CONNECTED);

      ESP_LOGI(TAG, "Connected");
      ESP_LOGI(TAG, "  Network IP %s", ipaddr_ntoa(&pppif->ip_addr));
      ESP_LOGI(TAG, "  Gateway IP %s", ipaddr_ntoa(&pppif->gw));
      ESP_LOGI(TAG, "  Netmask %s", ipaddr_ntoa(&pppif->netmask));

#if LWIP_DNS
      ESP_LOGI(TAG, "  DNS1 %s", ipaddr_ntoa(dns_getserver(0)));
      ESP_LOGI(TAG, "  DNS2 %s", ipaddr_ntoa(dns_getserver(1)));
#endif

      break;
    case PPPERR_PARAM:
      ESP_LOGE(TAG, "invalid parameter");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_OPEN:
      ESP_LOGE(TAG, "unable to open PPP session");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_DEVICE:
      ESP_LOGE(TAG, "Invalid I/O device for PPP");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_ALLOC:
      ESP_LOGE(TAG, "unable to allocate resources");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_USER:
      ESP_LOGE(TAG, "user interrupt");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_CONNECT:
      ESP_LOGE(TAG, "connection lost");
      global_pppos_component->set_state(PPPoSComponentState::CONNECTING);
      break;
    case PPPERR_AUTHFAIL:
      ESP_LOGE(TAG, "Failed authentication challenge");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_PROTOCOL:
      ESP_LOGE(TAG, "Failed to meet protocol");
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
    case PPPERR_PEERDEAD:
      ESP_LOGE(TAG, "Connection timeout");
      global_pppos_component->set_state(PPPoSComponentState::CONNECTING);
      break;
    case PPPERR_IDLETIMEOUT:
      ESP_LOGE(TAG, "Idle Timeout");
      global_pppos_component->set_state(PPPoSComponentState::CONNECTING);
      break;
    case PPPERR_CONNECTTIME:
      ESP_LOGE(TAG, "Max connect time reached");
      global_pppos_component->set_state(PPPoSComponentState::CONNECTING);
      break;
    case PPPERR_LOOPBACK:
      ESP_LOGE(TAG, "Loopback detected");
      global_pppos_component->set_state(PPPoSComponentState::CONNECTING);
      break;
    default:
      ESP_LOGE(TAG, "unknown error code: %d", err_code);
      global_pppos_component->set_state(PPPoSComponentState::STOPPED);
      global_pppos_component->mark_failed();
      break;
  }

  if (err_code == PPPERR_NONE) {
    return;
  }

  /* ppp_close() was previously called, don't reconnect */
  if (err_code == PPPERR_USER) {
    /* ppp_free(); -- can be called here */
    ppp_free(pcb);
    return;
  }

  if (!global_pppos_component->is_failed()) {
    ppp_connect(pcb, 30);  // try to reconnect in 30s
    // ppp_listen(pcb);
  }
}

uint32_t PPPoSComponent::output_cb(ppp_pcb *pcb, uint8_t *data, uint32_t len, void *ctx) {
  LWIP_UNUSED_ARG(pcb);
  LWIP_UNUSED_ARG(ctx);
  global_pppos_component->write_array(data, len);
  // global_pppos_component->flush();
  return len;
}

void PPPoSComponent::loop() {
  const size_t len = this->available();
  if (len > 0) {
    uint8_t data[len];
    if (!this->read_array(data, len)) {
      ESP_LOGE(TAG, "error read_array");
    }
    ESP_LOGV(TAG, "receiving %d bytes", len);
    // pppos_input(this->ppp_control_block_, data, len);
    pppos_input_tcpip(this->ppp_control_block_, data, len);
    // global_pppos_component->flush();
  }
  sys_check_timeouts();
}

void PPPoSComponent::dump_config() { ESP_LOGCONFIG(TAG, "PPPoS:"); }

void PPPoSComponent::set_state(PPPoSComponentState new_state) {
  if (new_state != this->state_)
    ESP_LOGI(TAG, "State update: %s -> %s", state_to_string(this->state_).c_str(), state_to_string(new_state).c_str());
  this->state_ = new_state;
}

std::string PPPoSComponent::state_to_string(PPPoSComponentState state) {
  switch (state) {
    case PPPoSComponentState::STOPPED:
      return "STOPPED";
    case PPPoSComponentState::CONNECTING:
      return "CONNECTING";
    case PPPoSComponentState::CONNECTED:
      return "CONNECTED";
  }
  return "UNKNOWN";
}

}  // namespace ppp
}  // namespace esphome
