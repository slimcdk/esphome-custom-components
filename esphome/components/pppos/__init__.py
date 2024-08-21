from esphome.core import CORE, coroutine_with_priority
from esphome.components import uart
from esphome.components.esp32 import add_idf_sdkconfig_option
from esphome.components.network import IPAddress

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv

from esphome.const import (
    CONF_ID,
    CONF_USERNAME,
    CONF_PASSWORD,
    CONF_MDNS,
    CONF_DISABLED,
    CONF_GATEWAY,
    CONF_SUBNET,
    CONF_DNS1,
    CONF_DNS2
)

CONF_PPPOS_ID = "pppos_id"
CONF_APN = "apn"
CONF_DNS = "dns"

CODEOWNERS = ["@slimcdk"]

MULTI_CONF = False
# CONFLICTS_WITH = ["wifi", "ethernet"]
AUTO_LOAD = ["network"]
DEPENDENCIES = ["uart"]

ppp_ns = cg.esphome_ns.namespace("ppp")
PPPoSComponent = ppp_ns.class_("PPPoSComponent", cg.Component, uart.UARTDevice)
NetworkDNS = ppp_ns.struct("NetworkDNS")

NETWORK_DNS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DNS1, default="0.0.0.0"): cv.ipv4,
        cv.Optional(CONF_DNS2, default="0.0.0.0"): cv.ipv4,
    }
)


CONFIG_SCHEMA = cv.All(
    cv.only_with_esp_idf,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PPPoSComponent),
            cv.Optional(CONF_APN): cv.string,
            cv.Optional(CONF_USERNAME): cv.string,
            cv.Optional(CONF_PASSWORD): cv.string,
            cv.Optional(CONF_DNS): NETWORK_DNS_SCHEMA,
        }
    ).extend(uart.UART_DEVICE_SCHEMA),
)

def network_dns(config):
    return cg.StructInitializer(
        NetworkDNS,
        ("dns1", IPAddress(*config[CONF_DNS1].args)),
        ("dns2", IPAddress(*config[CONF_DNS2].args)),
    )


@coroutine_with_priority(60.0)
async def to_code(config):

    cg.add_define("USE_PPPOS")
    cg.add_define("PPP_SUPPORT")
    cg.add_define("PAP_SUPPORT")
    cg.add_define("CHAP_SUPPORT")
    cg.add_define("MSCHAP_SUPPORT")
    # add_idf_sdkconfig_option("sys_jiffies", "millis")  # ppp uses this for randomness
    # add_idf_sdkconfig_option("CONFIG_PPP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_PPP_PAP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_PPP_CHAP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_PPP_MSCHAP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_LWIP_NETIF_API", True)
    # add_idf_sdkconfig_option("CONFIG_LWIP_DNS_SUPPORT_MDNS_QUERIES", False)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_DNS in config:
        cg.add(var.set_network_dns(network_dns(config[CONF_DNS])))


def final_validate(config):
    if not fv.full_config.get()[CONF_MDNS][CONF_DISABLED]:
        raise cv.Invalid("mdns must be disabled with cellular connection")
    return config


FINAL_VALIDATE_SCHEMA = cv.All(
    uart.final_validate_device_schema("pppos", require_tx=True, require_rx=True),
    final_validate,
)
