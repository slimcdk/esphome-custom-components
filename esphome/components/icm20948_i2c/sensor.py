import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import icm20948, i2c
from esphome.const import CONF_ID

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["i2c"]

AUTO_LOAD = ["icm20948"]

_LOGGER = logging.getLogger(__name__)

icm20948_i2c_ns = cg.esphome_ns.namespace("icm20948_i2c")
ICM20948I2CComponent = icm20948_i2c_ns.class_(
    "ICM20948I2C", icm20948.ICM20948Component, i2c.I2CDevice
)

# AD0 = 0 -> 1101000
# AD0 = 1 -> 1101001

CONFIG_SCHEMA = icm20948.ICM20948_BASE_SCHEMA.extend(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(ICM20948I2CComponent),
        }
    )
).extend(i2c.i2c_device_schema(0x68))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await icm20948.register_icm20948_device(var, config)
    await i2c.register_i2c_device(var, config)
