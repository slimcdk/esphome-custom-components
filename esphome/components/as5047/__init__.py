"""
Based of https://github.com/Adrien-Legrand/AS5047
"""

from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import (
    CONF_ID,
)

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["spi"]
MULTI_CONF = True

CONF_AS5047_ID = "as5047_id"
CONF_ABI_PULSE_PER_REVOLUTION = "ab_ppr"


as5047_ns = cg.esphome_ns.namespace("as5047")
AS5047Component = as5047_ns.class_("AS5047Component", cg.Component, spi.SPIDevice)
# AS5047ComponentConfigureAction = as5047_ns.class_(
#     "AS5047ComponentConfigureAction", automation.Action
# )


DEVICE_SCHEMA = cv.Schema({cv.GenerateID(CONF_AS5047_ID): cv.use_id(AS5047Component)})


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS5047Component),
        }
    ).extend(
        cv.COMPONENT_SCHEMA,
        spi.spi_device_schema(),
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)


# @automation.register_action(
#     "as5047.configure",
#     AS5047ComponentConfigureAction,
#     cv.Schema(
#         {
#             cv.GenerateID(): cv.use_id(AS5047Component),
#             cv.Optional(CONF_ABI_PULSE_PER_REVOLUTION): cv.templatable(
#                 cv.one_of(500, 400, 300, 200, 100, 50, 25, 8, 512, 256)
#             ),
#         }
#     ),
# )
# def tmc2209_configure_to_code(config, action_id, template_arg, args):
#     var = cg.new_Pvariable(action_id, template_arg)
#     yield cg.register_parented(var, config[CONF_ID])

#     if CONF_ABI_PULSE_PER_REVOLUTION in config:
#         template_ = yield cg.templatable(
#             config[CONF_ABI_PULSE_PER_REVOLUTION], args, int
#         )
#         cg.add(var.set_ab_pulses_pr_revolution(template_))

#     yield var
