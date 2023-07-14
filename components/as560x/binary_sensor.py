import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import AS560XComponent, CONF_AS560X_ID

DEPENDENCIES = ["as560x"]

CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(CONF_AS560X_ID): cv.use_id(AS560XComponent),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AS560X_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(hub.set_presence_binary_sensor(var))
