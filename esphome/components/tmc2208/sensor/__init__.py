from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_MILLIAMP,
    ICON_PERCENT,
)
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from .. import tmc2208_ns, TMC2208Component, DEVICE_SCHEMA, CONF_TMC2208_ID

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc2208"]

sensor_base = (
    cg.PollingComponent,
    sensor.Sensor,
    cg.Parented.template(TMC2208Component),
)

StallGuardResultSensor = tmc2208_ns.class_("StallGuardResultSensor", *sensor_base)
MotorLoadSensor = tmc2208_ns.class_("MotorLoadSensor", *sensor_base)
ActualCurrentSensor = tmc2208_ns.class_("ActualCurrentSensor", *sensor_base)
PWMScaleSumSensor = tmc2208_ns.class_("PWMScaleSumSensor", *sensor_base)
PWMScaleAutoSensor = tmc2208_ns.class_("PWMScaleAutoSensor", *sensor_base)
PWMOFSAutoSensor = tmc2208_ns.class_("PWMOFSAutoSensor", *sensor_base)
PWMGradAutoSensor = tmc2208_ns.class_("PWMGradAutoSensor", *sensor_base)


UNIT_MILLIVOLT = "mV"

TYPE_MOTOR_LOAD = "motor_load"
TYPE_STALLGUARD_RESULT = "stallguard_result"
TYPE_ACTUAL_CURRENT = "actual_current"
TYPE_PWM_SCALE_SUM = "pwm_scale_sum"
TYPE_PWM_SCALE_AUTO = "pwm_scale_auto"
TYPE_PWM_OFS_AUTO = "pwm_ofs_auto"
TYPE_PWM_GRAD_AUTO = "pwm_grad_auto"

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
        TYPE_PWM_SCALE_SUM: sensor.sensor_schema(
            PWMScaleSumSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_PWM_SCALE_AUTO: sensor.sensor_schema(
            PWMScaleAutoSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_PWM_OFS_AUTO: sensor.sensor_schema(
            PWMOFSAutoSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_PWM_GRAD_AUTO: sensor.sensor_schema(
            PWMGradAutoSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_TMC2208_ID])
