import logging

from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_PLATFORM
)
import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import uart


_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@slimcdk"]

MULTI_CONF = True

CONF_TMC2208 = "tmc2208"
CONF_TMC2208_HUB = "tmc2208_hub"
CONF_TMC2208_HUB_ID = "tmc2208_hub_id"

CONF_STEPPER = "stepper"

tmc2208_hub_ns = cg.esphome_ns.namespace("tmc2208_hub")
TMC2208Hub = tmc2208_hub_ns.class_("TMC2208Hub", cg.Component, uart.UARTDevice)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC2208Hub),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


TMC2208_HUB_DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2208_HUB_ID): cv.use_id(TMC2208Hub),
    }
)


async def register_tmc2208_hub_device(var, config):
    parent = await cg.get_variable(config[CONF_TMC2208_HUB_ID])
    await cg.register_parented(var, parent)

    # make hub aware of referenced instances
    cg.add(parent.add_device_to_hub_(str(config[CONF_ID]), config[CONF_ADDRESS]))


def final_validate(config):

    full_config = fv.full_config.get()
    steppers_in_hub = [
        stepper
        for stepper in full_config.get(CONF_STEPPER, [])
        if stepper[CONF_PLATFORM] == CONF_TMC2208
        and CONF_TMC2208_HUB_ID in stepper
        and stepper[CONF_TMC2208_HUB_ID] == config[CONF_ID]
    ]



    for i, stepper in enumerate(steppers_in_hub):
        for j in range(i + 1, len(steppers_in_hub)):
            if stepper[CONF_ADDRESS] == steppers_in_hub[j][CONF_ADDRESS]:
                _LOGGER.error(
                    'TMC2208 steppers "%s" and "%s" have overlapping addresses which will conflict',
                    stepper[CONF_ID],
                    steppers_in_hub[j][CONF_ID],
                )

    return uart.final_validate_device_schema(
        CONF_TMC2208_HUB, require_rx=True, require_tx=True
    )(config)


FINAL_VALIDATE_SCHEMA = final_validate
