import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, voltage_sampler
from esphome.const import (
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from ..stepper import tmc5240_ns, Stepper, DEVICE_SCHEMA, CONF_TMC5240_ID

common_sensor = (cg.PollingComponent, sensor.Sensor, cg.Parented.template(Stepper))

TemperatureSensor = tmc5240_ns.class_("TemperatureSensor", *common_sensor)
VoltageSensor = tmc5240_ns.class_("VoltageSensor", *common_sensor)
EncoderPosSensor = tmc5240_ns.class_("EncoderPosSensor", *common_sensor)
StallGuardResultSensor = tmc5240_ns.class_("StallGuardResultSensor", *common_sensor)

UNIT_MILLIVOLT = "mV"

TYPE_TEMPERATURE = "temperature"
TYPE_VOLTAGE = "voltage"
TYPE_ENCODER_POS = "encoder_pos"
TYPE_STALLGUARD_RESULT = "stallguard_result"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_TEMPERATURE: sensor.sensor_schema(
            TemperatureSensor,
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("30s")),
        TYPE_VOLTAGE: sensor.sensor_schema(
            VoltageSensor,
            unit_of_measurement=UNIT_MILLIVOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("60s")),
        TYPE_ENCODER_POS: sensor.sensor_schema(
            EncoderPosSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("60s")),
        TYPE_STALLGUARD_RESULT: sensor.sensor_schema(
            StallGuardResultSensor,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(DEVICE_SCHEMA, cv.polling_component_schema("60s")),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_TMC5240_ID])
