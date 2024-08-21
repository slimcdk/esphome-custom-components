from esphome.core import CORE, coroutine_with_priority
from esphome.components import uart
from esphome.components.esp32 import add_idf_sdkconfig_option

import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.const import (
    CONF_ID,
)

CONF_APN = "apn"

CODEOWNERS = ["@slimcdk"]

MULTI_CONF = False
DEPENDENCIES = ["uart"]

modem_ns = cg.esphome_ns.namespace("ppp")
ModemComponent = modem_ns.class_("ModemComponent", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModemComponent),
            cv.Optional(CONF_APN): cv.string,
        }
    ).extend(uart.UART_DEVICE_SCHEMA),
)

@coroutine_with_priority(60.0)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

