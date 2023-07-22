from esphome import automation, pins
from esphome.components import stepper, uart, binary_sensor
from esphome.components.stepper import validate_speed
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_DIRECTION,
    CONF_CHANNEL,
    CONF_ADDRESS,
    CONF_ID,
    CONF_ENABLE_PIN,
    CONF_CURRENT,
)


tmc_ns = cg.esphome_ns.namespace("tmc")
TMC2209 = tmc_ns.class_("TMC2209", stepper.Stepper, cg.Component)
TMC2209SetupAction = tmc_ns.class_("TMC2209SetupAction", automation.Action)

CONF_VELOCITY = "velocity"
CONF_MICROSTEPS = "microsteps"
CONF_TCOOL_THRESHOLD = "tcool_threshold"
CONF_STALL_THRESHOLD = "stall_threshold"
CONF_DIAG_PIN = "diagnostics_pin"
CONF_DIAG = "diagnostics"

CONFIG_SCHEMA = (
    stepper.STEPPER_SCHEMA.extend(
        {
            cv.Required(CONF_ID): cv.declare_id(TMC2209),
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_DIAG): binary_sensor.binary_sensor_schema,
            cv.Optional(CONF_CHANNEL): cv.int_,
            cv.Optional(CONF_ADDRESS): cv.i2c_address,
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)
    await uart.register_uart_device(var, config)

    if CONF_ENABLE_PIN in config:
        enable_pin = await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])
        cg.add(var.set_enable_pin(enable_pin))

    cg.add_library(
        "https://github.com/slimcdk/TMC-API", "3.5.1"
    )  # fork of https://github.com/trinamic/TMC-API with platformio library indexing


@automation.register_action(
    "tmc2209.setup",
    TMC2209SetupAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TMC2209),
            cv.Optional(CONF_DIRECTION): cv.templatable(cv.boolean),
            cv.Optional(CONF_VELOCITY): cv.templatable(cv.int_),
            cv.Optional(CONF_CURRENT): cv.templatable(cv.int_),
        }
    ),
)
def tmc2209_setup_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    if CONF_DIRECTION in config:
        template_ = yield cg.templatable(config[CONF_DIRECTION], args, bool)
        cg.add(var.set_direction(template_))

    if CONF_VELOCITY in config:
        template_ = yield cg.templatable(config[CONF_VELOCITY], args, int)
        cg.add(var.set_velocity(template_))

    if CONF_CURRENT in config:
        template_ = yield cg.templatable(config[CONF_CURRENT], args, int)
        cg.add(var.set_current(template_))

    yield var
