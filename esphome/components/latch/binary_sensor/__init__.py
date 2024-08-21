import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import binary_sensor
from esphome.const import CONF_ID, CONF_PIN

from esphome.automation import maybe_simple_id

import esphome.codegen as cg

CODEOWNERS = ["@slimcdk"]
latch_ns = cg.esphome_ns.namespace("latch")


CONF_AUTO_RESET = "auto_reset"

GPIOSRLatchBinarySensor = latch_ns.class_(
    "SRLatch", binary_sensor.BinarySensor, cg.Component
)
GPIOSRLatchBinarySensorResetAction = latch_ns.class_(
    "GPIOSRLatchBinarySensorResetAction", automation.Action
)

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(GPIOSRLatchBinarySensor)
    .extend(
        {
            cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_AUTO_RESET, default=False): bool,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    if config[CONF_AUTO_RESET]:
        cg.add(var.set_auto_reset(config[CONF_AUTO_RESET]))
        # cg.define("USE_AUTO_RESET")

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))


# @automation.register_action(
#     "latch.reset",
#     GPIOSRLatchBinarySensorResetAction,
#     maybe_simple_id(
#         {
#             cv.Required(CONF_ID): cv.use_id(GPIOSRLatchBinarySensor),
#         }
#     ),
# )
# async def sr_latch_configure_to_code(config, action_id, template_arg, args):
#     paren = await cg.get_variable(config[CONF_ID])
#     return cg.new_Pvariable(action_id, template_arg, paren)
