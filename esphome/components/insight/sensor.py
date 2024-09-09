import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_FREQUENCY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_HERTZ,
    ICON_PULSE,
)

CODEOWNERS = ["@slimcdk"]

insight_ns = cg.esphome_ns.namespace("insight")
ESPHomeLoopSensor = insight_ns.class_(
    "ESPHomeLoopSensor", cg.PollingComponent, sensor.Sensor
)

TYPE_ESPHOME_LOOP = "esphome_loop"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_ESPHOME_LOOP: sensor.sensor_schema(
            ESPHomeLoopSensor,
            unit_of_measurement=UNIT_HERTZ,
            device_class=DEVICE_CLASS_FREQUENCY,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon=ICON_PULSE,
        ).extend(cv.polling_component_schema("10s"))
    },
    default_type=TYPE_ESPHOME_LOOP,
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
