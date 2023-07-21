import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_VARIANT

from esphome import automation


CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["sensor", "binary_sensor"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_AS5601 = "as5601"
CONF_AS5600 = "as5600"
CONF_AS560X_ID = "as560x_id"
CONF_ANGLE_ZERO_POSITION = "zero_angle_position"
CONF_AB_RESOLUTION = "ab_resolution"
CONF_PUSH_THRESHOLD = "push_threshold"
CONF_ANGLE_STOP_POSITION = "angle_stop_position"
CONF_MAXIMUM_ANGLE = "maximum_angle"

_CONF_COMPONENT = "component"
_CONF_CONFS_NOT_ALLOWED = "confs_not_allowed"


SCHEMA_ANGLE_ZERO_POSITION = cv.int_range(
    min=0, max=2**12 - 1, min_included=True, max_included=False
)
SCHEMA_ANGLE_STOP_POSITION = cv.int_
SCHEMA_MAXIMUM_ANGLE = cv.int_
SCHEMA_AB_RESOLUTION = cv.one_of(*[2**i for i in range(3, 12)])  # 8,16,...,1024,2048
SCHEMA_PUSH_THRESHOLD = cv.int_

as560x_ns = cg.esphome_ns.namespace("as560x")
AS560XComponent = as560x_ns.class_("AS560XComponent", cg.Component, i2c.I2CDevice)
AS5600Component = as560x_ns.class_("AS5600Component", AS560XComponent)
AS5601Component = as560x_ns.class_("AS5601Component", AS560XComponent)

AS5600ConfigAction = as560x_ns.class_("AS5600ConfigAction", automation.Action)
AS5601ConfigAction = as560x_ns.class_("AS5601ConfigAction", automation.Action)


VARIANT_COMP_MAP = {
    CONF_AS5600: {
        _CONF_COMPONENT: AS5600Component,
        _CONF_CONFS_NOT_ALLOWED: [CONF_AB_RESOLUTION, CONF_PUSH_THRESHOLD],
    },
    CONF_AS5601: {
        _CONF_COMPONENT: AS5601Component,
        _CONF_CONFS_NOT_ALLOWED: [CONF_ANGLE_STOP_POSITION, CONF_MAXIMUM_ANGLE],
    },
}


def set_variant_comp(config):
    new_component = VARIANT_COMP_MAP[config[CONF_VARIANT]][_CONF_COMPONENT]
    config[CONF_ID] = cv.declare_id(new_component)(config[CONF_ID].id)
    return config


def validate_variant_confs(config):
    for conf in VARIANT_COMP_MAP[config[CONF_VARIANT]][_CONF_CONFS_NOT_ALLOWED]:
        if conf in config:
            raise cv.Invalid(
                f"{conf} must not be specified with {config[CONF_VARIANT]}"
            )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS560XComponent),
            cv.Required(CONF_VARIANT): cv.one_of(CONF_AS5600, CONF_AS5601, lower=True),
            cv.Optional(CONF_ANGLE_ZERO_POSITION): SCHEMA_ANGLE_ZERO_POSITION,
            # AS5600
            cv.Optional(CONF_ANGLE_STOP_POSITION): SCHEMA_ANGLE_STOP_POSITION,
            cv.Optional(CONF_MAXIMUM_ANGLE): SCHEMA_MAXIMUM_ANGLE,
            # AS5601
            cv.Optional(CONF_AB_RESOLUTION): SCHEMA_AB_RESOLUTION,
            cv.Optional(CONF_PUSH_THRESHOLD): SCHEMA_PUSH_THRESHOLD,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36)),
    set_variant_comp,
    validate_variant_confs,
)


@automation.register_action(
    "as5600.config",
    AS5600ConfigAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(AS5600Component),
            cv.Optional(CONF_ANGLE_ZERO_POSITION): cv.templatable(
                SCHEMA_ANGLE_ZERO_POSITION
            ),
            cv.Optional(CONF_ANGLE_STOP_POSITION): cv.templatable(
                SCHEMA_ANGLE_STOP_POSITION
            ),
            cv.Optional(CONF_MAXIMUM_ANGLE): cv.templatable(SCHEMA_MAXIMUM_ANGLE),
        }
    ),
)
async def as5600_config_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    if CONF_ANGLE_ZERO_POSITION in config:
        template_ = await cg.templatable(
            config[CONF_ANGLE_ZERO_POSITION], args, cg.uint16
        )
        cg.add(var.set_zero_position(template_))
    if CONF_ANGLE_STOP_POSITION in config:
        template_ = await cg.templatable(
            config[CONF_ANGLE_STOP_POSITION], args, cg.uint16
        )
        cg.add(var.set_stop_position(template_))
    if CONF_ANGLE_STOP_POSITION in config:
        template_ = await cg.templatable(
            config[CONF_ANGLE_STOP_POSITION], args, cg.uint16
        )
        cg.add(var.set_maximum_angle(template_))
    return var


@automation.register_action(
    "as5601.config",
    AS5601ConfigAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(AS5601Component),
            cv.Optional(CONF_ANGLE_ZERO_POSITION): cv.templatable(
                CONF_ANGLE_ZERO_POSITION
            ),
            cv.Optional(CONF_AB_RESOLUTION): cv.templatable(SCHEMA_AB_RESOLUTION),
            cv.Optional(CONF_PUSH_THRESHOLD): cv.templatable(SCHEMA_PUSH_THRESHOLD),
        }
    ),
)
async def as5601_config_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    if CONF_ANGLE_ZERO_POSITION in config:
        template_ = await cg.templatable(
            config[CONF_ANGLE_ZERO_POSITION], args, cg.uint16
        )
        cg.add(var.set_zero_position(template_))
    if CONF_AB_RESOLUTION in config:
        template_ = await cg.templatable(config[CONF_AB_RESOLUTION], args, cg.uint16)
        cg.add(var.set_ab_resolution(template_))
    if CONF_PUSH_THRESHOLD in config:
        template_ = await cg.templatable(config[CONF_PUSH_THRESHOLD], args, cg.uint16)
        cg.add(var.set_push_threshold(template_))
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_ANGLE_ZERO_POSITION in config:
        cg.add(var.set_zero_position(config[CONF_ANGLE_ZERO_POSITION]))

    if config[CONF_VARIANT] == CONF_AS5600:
        if CONF_ANGLE_STOP_POSITION in config:
            cg.add(var.set_stop_position(config[CONF_ANGLE_STOP_POSITION]))
        if CONF_ANGLE_STOP_POSITION in config:
            cg.add(var.set_maximum_angle(config[CONF_ANGLE_STOP_POSITION]))

    if config[CONF_VARIANT] == CONF_AS5601:
        if CONF_AB_RESOLUTION in config:
            cg.add(var.set_ab_resolution(config[CONF_AB_RESOLUTION]))
        if CONF_PUSH_THRESHOLD in config:
            cg.add(var.set_push_threshold(config[CONF_PUSH_THRESHOLD]))


## Example configuration
# esphome:
#   on_boot:
#     - as5601.config:
#         ab_resolution: 2048

# i2c:
#     sda: GPIO21
#     scl: GPIO22

# as560x:
#   - id: main_as5601
#     variant: as5601
#     ab_resolution: 1024
#     zero_angle_position: 0
#     angle_sensor:
#         name: AS5600 Magnet Angle
#         id: as5600_magnet_angle
#     magnitude_sensor:
#         id: as5600_magnet_field_strength
#         name: AS5600 Magnet Field Strength

# binary_sensor:
#   - platform: gpio
#     id: as5600_magnet_detected
#     name: AS5600 Magnet detected
#     pin: ...
