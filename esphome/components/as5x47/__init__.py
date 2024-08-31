"""
Based of https://github.com/Adrien-Legrand/AS5X47
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

CONF_AS5X47_ID = "as5x47_id"
CONF_ABI_PULSE_PER_REVOLUTION = "ab_ppr"


as5x47_ns = cg.esphome_ns.namespace("as5x47")
AS5X47Component = as5x47_ns.class_("AS5X47Component", cg.Component, spi.SPIDevice)
# AS5X47ComponentConfigureAction = as5x47_ns.class_(
#     "AS5X47ComponentConfigureAction", automation.Action
# )


DEVICE_SCHEMA = cv.Schema({cv.GenerateID(CONF_AS5X47_ID): cv.use_id(AS5X47Component)})


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS5X47Component),
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
#     "as5x47.configure",
#     AS5X47ComponentConfigureAction,
#     cv.Schema(
#         {
#             cv.GenerateID(): cv.use_id(AS5X47Component),
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
