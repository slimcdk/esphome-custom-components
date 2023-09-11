import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, voltage_sampler
from esphome.const import CONF_ID, CONF_NUMBER
from ..stepper.stepper import tmc_ns, TMC2209


DEPENDENCIES = ["stepper.tmc2209"]

TMC2209Sensor = tmc_ns.class_("TMC2209Sensor", sensor.Sensor)
CONF_TMC2209_ID = "tmc2209_id"

CONFIG_SCHEMA = (
    sensor.sensor_schema(TMC2209Sensor)
    .extend(
        {
            cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209),
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_parented(var, config[CONF_TMC2209_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
