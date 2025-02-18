import logging

from esphome.const import CONF_ID
import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import uart


_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@slimcdk"]

MULTI_CONF = True

CONF_TMC22XX_HUB = "tmc22xx_hub"
CONF_TMC22XX_HUB_ID = "tmc22xx_hub_id"

CONF_STEPPER = "stepper"

tmc22xx_hub = cg.esphome_ns.namespace("tmc22xx_hub")
TMC22XXHub = tmc22xx_hub.class_("TMC22XXHub", cg.Component, uart.UARTDevice)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC22XXHub),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


TMC22XX_HUB_DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC22XX_HUB_ID): cv.use_id(TMC22XXHub),
    }
)


async def register_tmc22xx_hub_device(var, config):
    parent = await cg.get_variable(config[CONF_TMC22XX_HUB_ID])
    await cg.register_parented(var, parent)
    return parent


def final_validate(config):

    # full_config = fv.full_config.get()
    # steppers_in_hub = [
    #     stepper
    #     for stepper in full_config.get(CONF_STEPPER, [])
    #     if stepper[CONF_TMC22XX_HUB_ID] == config[CONF_ID]
    # ]

    # for i, stepper in enumerate(steppers_in_hub):
    #     for j in range(i + 1, len(steppers_in_hub)):
    #         if stepper[CONF_ADDRESS] == steppers_in_hub[j][CONF_ADDRESS]:
    #             _LOGGER.error(
    #                 'TMC22XX steppers "%s" and "%s" have overlapping addresses which will conflict',
    #                 stepper[CONF_ID],
    #                 steppers_in_hub[j][CONF_ID],
    #             )

    return uart.final_validate_device_schema(
        CONF_TMC22XX_HUB, require_rx=True, require_tx=True
    )(config)


FINAL_VALIDATE_SCHEMA = final_validate
