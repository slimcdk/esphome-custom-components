import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    ICON_PERCENT,
)

from .. import tmc2209_ns, TMC2209, DEVICE_SCHEMA, CONF_TMC2209_ID

CODEOWNERS = ["@slimcdk"]

common_sensor = (cg.PollingComponent, sensor.Sensor, cg.Parented.template(TMC2209))
StallGuardResultSensor = tmc2209_ns.class_("StallGuardResultSensor", *common_sensor)
MotorLoadSensor = tmc2209_ns.class_("MotorLoadSensor", *common_sensor)

UNIT_MILLIVOLT = "mV"

TYPE_STALLGUARD_RESULT = "stallguard_result"
TYPE_MOTOR_LOAD = "motor_load"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_STALLGUARD_RESULT: sensor.sensor_schema(
            StallGuardResultSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_MOTOR_LOAD: sensor.sensor_schema(
            MotorLoadSensor,
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_PERCENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_TMC2209_ID])
