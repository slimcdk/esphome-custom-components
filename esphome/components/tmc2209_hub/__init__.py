import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import (
    CONF_ID,
)

CODEOWNERS = ["@slimcdk"]

CONF_TMC2209_HUB = "tmc2209_hub"
CONF_TMC2209_HUB_ID = "tmc2209_hub_id"

MULTI_CONF = True
# MULTI_CONF_NO_DEFAULT = True

tmc2209_hub_ns = cg.esphome_ns.namespace("tmc2209_hub")
TMC2209Hub = tmc2209_hub_ns.class_("TMC2209Hub", cg.Component, uart.UARTDevice)
TMC2209Device = tmc2209_hub_ns.class_("TMC2209Device")

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC2209Hub),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


TMC2209_DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2209_HUB_ID): cv.use_id(TMC2209Hub),
    }
)


async def register_tmc2209_device(var, config):
    parent = await cg.get_variable(config[CONF_TMC2209_HUB_ID])
    cg.add(var.set_tmc2209_hub_parent(parent))


TMC2209_HUB_FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    CONF_TMC2209_HUB, require_rx=True, require_tx=True
)
