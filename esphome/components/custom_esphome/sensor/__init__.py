import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_FREQUENCY,
    UNIT_HERTZ,
    ICON_PULSE,
)

CODEOWNERS = ["@slimcdk"]

loop_frequency_ns = cg.esphome_ns.namespace("loop_frequency")
LoopFrequencySensor = loop_frequency_ns.class_(
    "LoopFrequencySensor", cg.PollingComponent, sensor.Sensor
)

TYPE_LOOP_FREQUENCY = "loop_frequency"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_LOOP_FREQUENCY: sensor.sensor_schema(
            LoopFrequencySensor,
            unit_of_measurement=UNIT_HERTZ,
            device_class=DEVICE_CLASS_FREQUENCY,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_PULSE,
        ).extend(cv.polling_component_schema("10s"))
    },
    default_type=TYPE_LOOP_FREQUENCY,
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
