import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    ICON_PERCENT,
)

from .. import as5047_ns, AS5047Component, DEVICE_SCHEMA, CONF_AS5047_ID

CODEOWNERS = ["@slimcdk"]

common_sensor = (
    cg.PollingComponent,
    sensor.Sensor,
    cg.Parented.template(AS5047Component),
)
AngleSensor = as5047_ns.class_("AngleSensor", *common_sensor)


TYPE_ANGLE = "angle"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_ANGLE: sensor.sensor_schema(
            AngleSensor,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_AS5047_ID])
