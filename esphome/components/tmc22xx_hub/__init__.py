import logging

from esphome.const import CONF_ID, CONF_ADDRESS
import esphome.codegen as cg
import esphome.config as c
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import uart
from esphome.core.entity_helpers import inherit_property_from


_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@slimcdk"]

MULTI_CONF = True

CONF_TMC22XX_HUB = "tmc22xx_hub"
CONF_TMC22XX_HUB_ID = "tmc22xx_hub_id"

CONF_STEPPER = "stepper"
CONF_TMC22XX = "tmc22xx"
CONF_SELECT_PIN = "select_pin"

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


def _final_validate(config):

    # find all tmc22xx components with reference to this hub
    hub_id = config.get(CONF_ID)
    fc = fv.full_config.get()
    components = list(fc.get(CONF_STEPPER, [])) + list(fc.get(CONF_TMC22XX, []))
    children = [c for c in components if c.get(CONF_TMC22XX_HUB_ID) == hub_id]

    for i, c1 in enumerate(children):
        for _, c2 in enumerate(children[i + 1 :]):
            using_select_pin = CONF_SELECT_PIN in c1 and CONF_SELECT_PIN in c2
            has_same_addr = c1.get(CONF_ADDRESS) == c2.get(CONF_ADDRESS)
            if not using_select_pin and has_same_addr:
                _LOGGER.error(
                    "TMC22XX `%s` and `%s` have conflicting addresses",
                    c1.get(CONF_ID),
                    c2.get(CONF_ID),
                )

    return uart.final_validate_device_schema(
        CONF_TMC22XX_HUB, require_rx=True, require_tx=True
    )(config)


FINAL_VALIDATE_SCHEMA = _final_validate


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
