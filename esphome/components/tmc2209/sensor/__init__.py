import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from .. import (
    tmc_ns,
    TMC2209,
    CONF_STALLGUARD_RESULT
)

from ..stepper import (
    TMC2209_STEPPER_SCHEMA
)


AUTO_LOAD = ["tmc2209", "stepper"]

TMC2209Sensor = tmc_ns.class_("TMC2209Sensor", TMC2209, sensor.Sensor, cg.PollingComponent)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC2209Sensor),
            cv.Required(CONF_STALLGUARD_RESULT): sensor.sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(TMC2209_STEPPER_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # await sensor.register_sensor(var, config)

    if CONF_STALLGUARD_RESULT in config:
        sens = await sensor.new_sensor(config[CONF_STALLGUARD_RESULT])
        cg.add(var.set_stallguard_result_sensor(sens))

