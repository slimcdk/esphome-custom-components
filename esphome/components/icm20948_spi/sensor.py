import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import icm20948, spi
from esphome.const import CONF_ID

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["spi"]

AUTO_LOAD = ["icm20948"]

_LOGGER = logging.getLogger(__name__)


icm20948_spi_ns = cg.esphome_ns.namespace("icm20948_spi")
ICM20948SPIComponent = icm20948_spi_ns.class_(
    "ICM20948SPI", icm20948.ICM20948Component, spi.SPIDevice
)

CONFIG_SCHEMA = icm20948.ICM20948_BASE_SCHEMA.extend(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(ICM20948SPIComponent),
        }
    )
).extend(spi.spi_device_schema(cs_pin_required=False))


async def to_code(config):
    _LOGGER.error("This component hasn't been tested")
    var = cg.new_Pvariable(config[CONF_ID])
    await icm20948.register_icm20948_device(var, config)
    await spi.register_spi_device(var, config)
