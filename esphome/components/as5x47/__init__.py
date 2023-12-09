'''
Based of https://github.com/Adrien-Legrand/AS5X47
'''

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

as5x47_ns = cg.esphome_ns.namespace("as5x47")
AS5X47Component = as5x47_ns.class_("AS5X47Component", cg.Component, spi.SPIDevice)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS5X47Component)
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema()),
)

async def to_code(config):
    var =  cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
