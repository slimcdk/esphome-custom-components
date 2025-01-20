import logging

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
)

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@slimcdk"]

MULTI_CONF = True

CONF_TMC2209_HUB = "tmc2209_hub"
CONF_TMC2209_HUB_ID = "tmc2209_hub_id"

CONF_STEPPER = "stepper"


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


def final_validate(config):

    full_config = fv.full_config.get()
    steppers_in_hub = [
        stepper
        for stepper in full_config[CONF_STEPPER]
        if stepper[CONF_TMC2209_HUB_ID] == config[CONF_ID]
    ]

    for i, stepper in enumerate(steppers_in_hub):
        for j in range(i + 1, len(steppers_in_hub)):
            if stepper[CONF_ADDRESS] == steppers_in_hub[j][CONF_ADDRESS]:
                _LOGGER.warning(
                    'TMC2209 steppers "%s" and "%s" have same address on same hub which will conflict',
                    stepper[CONF_ID],
                    steppers_in_hub[j][CONF_ID],
                )

    return uart.final_validate_device_schema(
        CONF_TMC2209_HUB, require_rx=True, require_tx=True
    )(config)


FINAL_VALIDATE_SCHEMA = final_validate
