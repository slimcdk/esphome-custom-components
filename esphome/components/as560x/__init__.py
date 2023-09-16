import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, binary_sensor
from esphome import automation
from esphome.const import (
    CONF_ID,
    UNIT_DEGREES,
    DEVICE_CLASS_MOTION
)

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = True

CONF_AS560X_ID = "as5601_id"

CONF_ANGLE_ZERO_POSITION = "zero_angle_position"
CONF_AB_RESOLUTION = "ab_resolution"
CONF_PUSH_THRESHOLD = "push_threshold"

CONF_MAGNITUDE_SENSOR = "magnitude"
CONF_ANGLE_SENSOR = "angle"
CONF_PRESENCE = "presence"

UNIT_MILLITESLA = "mT"

as560x_ns = cg.esphome_ns.namespace("as560x")
AS560X = as560x_ns.class_("AS560X", i2c.I2CDevice)
AS5600 = as560x_ns.class_("AS5600", cg.Component, AS560X)
AS5601 = as560x_ns.class_("AS5601", cg.Component, AS560X)

AS5601SetAction = as560x_ns.class_("AS5601SetAction", automation.Action)
AS5600SetAction = as560x_ns.class_("AS5600SetAction", automation.Action)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS5601),
            cv.Optional(CONF_MAGNITUDE_SENSOR): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLITESLA,
                accuracy_decimals=2,
            ),
            cv.Optional(CONF_ANGLE_SENSOR): sensor.sensor_schema(
                unit_of_measurement=UNIT_DEGREES,
                accuracy_decimals=2,
            ),
            cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_MOTION
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36)),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_MAGNITUDE_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_MAGNITUDE_SENSOR])
        cg.add(var.set_magnitude_sensor(sens))

    if CONF_ANGLE_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_SENSOR])
        cg.add(var.set_angle_sensor(sens))

    if CONF_PRESENCE in config:
        bin_sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_binary_sensor(bin_sens))


@automation.register_action(
    "as5600.set",
    AS5600SetAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(AS5600),
            cv.Optional(CONF_ANGLE_ZERO_POSITION): cv.templatable(cv.int_range(min=0, max=2**12 - 1, min_included=True, max_included=False)),
        }
    ),
)
async def as5600_config_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if CONF_ANGLE_ZERO_POSITION in config:
        template_ = await cg.templatable(config[CONF_ANGLE_ZERO_POSITION], args, cg.uint16)
        cg.add(var.set_zero_position(template_))

    return var



@automation.register_action(
    "as5601.set",
    AS5601SetAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(AS5601),
            cv.Optional(CONF_ANGLE_ZERO_POSITION): cv.templatable(cv.int_range(min=0, max=2**12 - 1, min_included=True, max_included=False)),
            cv.Optional(CONF_AB_RESOLUTION): cv.templatable(cv.int_range(min=0, max=2**12 - 1, min_included=True, max_included=False)),
            cv.Optional(CONF_PUSH_THRESHOLD): cv.templatable(cv.int_),
        }
    ),
)
async def as5601_config_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if CONF_ANGLE_ZERO_POSITION in config:
        template_ = await cg.templatable(config[CONF_ANGLE_ZERO_POSITION], args, cg.uint16)
        cg.add(var.set_zero_position(template_))

    if CONF_AB_RESOLUTION in config:
        template_ = await cg.templatable(config[CONF_AB_RESOLUTION], args, cg.uint16)
        cg.add(var.set_ab_resolution(template_))

    if CONF_PUSH_THRESHOLD in config:
        template_ = await cg.templatable(config[CONF_PUSH_THRESHOLD], args, cg.uint16)
        cg.add(var.set_push_threshold(template_))

    return var
