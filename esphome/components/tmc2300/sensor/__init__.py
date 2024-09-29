import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_MILLIAMP,
    ICON_PERCENT,
)

from ..stepper import tmc2300_ns, TMC2300, DEVICE_SCHEMA, CONF_TMC2300_ID

CODEOWNERS = ["@slimcdk"]

sensor_base = (cg.PollingComponent, sensor.Sensor, cg.Parented.template(TMC2300))
StallGuardResultSensor = tmc2300_ns.class_("StallGuardResultSensor", *sensor_base)
MotorLoadSensor = tmc2300_ns.class_("MotorLoadSensor", *sensor_base)
ActualCurrentSensor = tmc2300_ns.class_("ActualCurrentSensor", *sensor_base)

UNIT_MILLIVOLT = "mV"

TYPE_MOTOR_LOAD = "motor_load"
TYPE_STALLGUARD_RESULT = "stallguard_result"
TYPE_ACTUAL_CURRENT = "actual_current"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_MOTOR_LOAD: sensor.sensor_schema(
            MotorLoadSensor,
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_PERCENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_STALLGUARD_RESULT: sensor.sensor_schema(
            StallGuardResultSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_ACTUAL_CURRENT: sensor.sensor_schema(
            ActualCurrentSensor,
            unit_of_measurement=UNIT_MILLIAMP,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_TMC2300_ID])
