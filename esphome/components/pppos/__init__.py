from esphome.core import CORE, coroutine_with_priority
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_USERNAME,
    CONF_PASSWORD,
    CONF_VARIANT,
    CONF_MDNS,
    CONF_DISABLED,
)
from esphome.components import uart
from esphome.components.esp32 import add_idf_sdkconfig_option
import esphome.final_validate as fv


CODEOWNERS = ["@slimcdk"]

MULTI_CONF = False
# CONFLICTS_WITH = ["wifi", "ethernet"]
AUTO_LOAD = ["network"]

# NETWORK_AUTO_LOAD.remove(CONF_MDNS)
DEPENDENCIES = ["uart"]

ppp_ns = cg.esphome_ns.namespace("ppp")
PPPoSComponent = ppp_ns.class_("PPPoSComponent", cg.Component, uart.UARTDevice)

CONF_PPPOS_ID = "pppos_id"
CONF_APN = "apn"

CONFIG_SCHEMA = cv.All(
    cv.only_with_esp_idf,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PPPoSComponent),
            cv.Optional(CONF_APN): cv.string,
            cv.Optional(CONF_USERNAME): cv.string,
            cv.Optional(CONF_PASSWORD): cv.string,
        }
    ).extend(uart.UART_DEVICE_SCHEMA),
)


@coroutine_with_priority(60.0)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add_define("USE_PPPOS")

    add_idf_sdkconfig_option("sys_jiffies", "millis")  # ppp uses this for randomness
    add_idf_sdkconfig_option("CONFIG_PPP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_PPP_PAP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_PPP_CHAP_SUPPORT", True)
    # add_idf_sdkconfig_option("CONFIG_PPP_MSCHAP_SUPPORT", True)


def final_validate(config):
    if not fv.full_config.get()[CONF_MDNS][CONF_DISABLED]:
        raise cv.Invalid("mdns must be disabled with cellular connection")
    return config


FINAL_VALIDATE_SCHEMA = cv.All(
    uart.final_validate_device_schema("pppos", require_tx=True, require_rx=True),
    final_validate,
)
