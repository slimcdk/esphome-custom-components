#include "pppos_component.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/application.h"

#include <string.h>
#include <lwip/dns.h>
#include "esp_event.h"

namespace esphome {
namespace ppp {

static const char *TAG = "pppos.component";

#define ESPHL_ERROR_CHECK(err, message) \
  if ((err) != ESP_OK) { \
    ESP_LOGE(TAG, message ": (%d) %s", err, esp_err_to_name(err)); \
    this->mark_failed(); \
    return; \
  }

PPPoSComponent *global_pppos_component;

PPPoSComponent::PPPoSComponent() { global_pppos_component = this; }

void PPPoSComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PPPoS...");

  esp_err_t err;
  err = esp_netif_init();
  ESPHL_ERROR_CHECK(err, "netif init error");
  err = esp_event_loop_create_default();
  ESPHL_ERROR_CHECK(err, "event loop error");

  this->ppp_control_block_ = pppos_create(&this->ppp_netif_, PPPoSComponent::output_cb, this->status_cb, NULL);
  if (this->ppp_control_block_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create ppp control block");
    this->mark_failed();
    return;
  }

  ppp_set_default(this->ppp_control_block_);
  // ppp_set_usepeerdns(this->ppp_control_block_, 1);

  if (ppp_connect(this->ppp_control_block_, 10) != ERR_OK) {
    ESP_LOGE(TAG, "Couldnt initiate connection");
    this->mark_failed();
    return;
  }

  this->set_state(PPPoSComponentState::CONNECTING);
  ESP_LOGI(TAG, "Connecting...");
}

void PPPoSComponent::status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
  struct netif *pppif = ppp_netif(pcb);

  switch (err_code) {
    case PPPERR_NONE:
      global_pppos_component->set_state(PPPoSComponentState::CONNECTED);
      ESP_LOGI(TAG, "Connected");
      ESP_LOGI(TAG, "  WAN IP %s", ipaddr_ntoa(&pppif->ip_addr));
      ESP_LOGI(TAG, "  GW IP %s", ipaddr_ntoa(&pppif->gw));
      ESP_LOGI(TAG, "  Netmask %s", ipaddr_ntoa(&pppif->netmask));
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
    return;
  }

  // try to reconnect in 30s
  if (!global_pppos_component->is_failed()) {
    ppp_connect(pcb, 30);
  }
}
uint32_t PPPoSComponent::output_cb(ppp_pcb *pcb, uint8_t *data, uint32_t len, void *ctx) {
  ESP_LOGV(TAG, "cb: sending %d bytes", len);
  global_pppos_component->write_array((const uint8_t *) data, len);
  global_pppos_component->flush();
  return len;
}

void PPPoSComponent::loop() {
  sys_check_timeouts();

  if (this->available() > 0) {
    size_t size = this->available();
    uint8_t data[size];
    if (!this->read_array(data, size)) {
      ESP_LOGE(TAG, "error read_array");
    }
    ESP_LOGV(TAG, "receiving %d bytes", size);
    pppos_input(this->ppp_control_block_, data, size);
  }
}

void PPPoSComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "PPPoS:");

  ESP_LOGCONFIG(TAG, "  is_connected: %d", this->is_connected());
  ESP_LOGCONFIG(TAG, "  get_use_address: %s", this->get_use_address().c_str());
  ESP_LOGCONFIG(TAG, "  get_ip_address: %s", this->get_ip_address().str().c_str());

  // this->dump_connect_params_();
}

float PPPoSComponent::get_setup_priority() const { return setup_priority::WIFI; }

bool PPPoSComponent::can_proceed() { return this->is_connected() || this->is_failed(); }

network::IPAddress PPPoSComponent::get_ip_address() {
  // esp_netif_ip_info_t ip;
  // esp_netif_get_ip_info(this->ppp_netif_, &ip);
  // return {ip.ip.addr};
  return network::IPAddress(this->ppp_netif_.ip_addr.addr);
}

void PPPoSComponent::set_use_address(const std::string &use_address) { this->use_address_ = use_address; }

std::string PPPoSComponent::get_use_address() const {
  if (this->use_address_.empty()) {
    return App.get_name() + ".local";
  }
  return this->use_address_;
}

bool PPPoSComponent::is_connected() { return this->state_ == PPPoSComponentState::CONNECTED; }

void PPPoSComponent::set_manual_ip(const ManualIP &manual_ip) { this->manual_ip_ = manual_ip; }

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
