from esphome import pins
from esphome.components import stepper
from esphome.components import uart
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_DIR_PIN,
    CONF_ID,
    CONF_ENABLE_PIN,
    CONF_STEP_PIN,
)


tmc_ns = cg.esphome_ns.namespace("tmc")
TMC2209 = tmc_ns.class_("TMC2209", stepper.Stepper, cg.Component)

CONF_MICROSTEPS = "microsteps"
CONF_TCOOL_THRESHOLD = "tcool_threshold"
CONF_STALL_THRESHOLD = "stall_threshold"
CONF_DIAG_PIN = "diag_pin"

CONFIG_SCHEMA = (
    stepper.STEPPER_SCHEMA.extend(
        {
            cv.Required(CONF_ID): cv.declare_id(TMC2209),
            cv.Required(CONF_STEP_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
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

    step_pin = await cg.gpio_pin_expression(config[CONF_STEP_PIN])
    cg.add(var.set_step_pin(step_pin))

    dir_pin = await cg.gpio_pin_expression(config[CONF_DIR_PIN])
    cg.add(var.set_direction_pin(dir_pin))

    if CONF_ENABLE_PIN in config:
        enable_pin = await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])
        cg.add(var.set_enable_pin(enable_pin))

    if CONF_DIAG_PIN in config:
        diag_pin = await cg.gpio_pin_expression(config[CONF_DIAG_PIN])
        cg.add(var.set_diag_pin(diag_pin))

    cg.add_library(
        "https://github.com/slimcdk/TMC-API", "3.5.1"
    )  # fork of https://github.com/trinamic/TMC-API with platformio library indexing
