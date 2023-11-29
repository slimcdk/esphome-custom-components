from esphome import automation, pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.components import stepper

from esphome.const import CONF_PLATFORM

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["stepper"]
IS_PLATFORM_COMPONENT = False
CONF_STEPPER = "stepper"

CONF_TMC5240 = "tmc5240"
CONF_TMC5240_ID = "tmc5240_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG0_PIN = "diag0_pin"
CONF_DIAG1_PIN = "diag1_pin"


tmc5240_ns = cg.esphome_ns.namespace("tmc5240")
TMC5240 = tmc5240_ns.class_("TMC5240", cg.Component, stepper.Stepper)

TMC5240_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TMC5240),
        cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_DIAG0_PIN): pins.gpio_input_pin_schema,
        cv.Required(CONF_DIAG1_PIN): pins.gpio_input_pin_schema,
    },
).extend(cv.COMPONENT_SCHEMA).extend(stepper.STEPPER_SCHEMA)


async def setup_tmc5240_base(var, config):
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)

    enn_pin = await cg.gpio_pin_expression(config[CONF_ENN_PIN])
    cg.add(var.set_enn_pin(enn_pin))

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.5.2")


def final_validate_config(config):
    steppers = fv.full_config.get()[CONF_STEPPER]
    tmc5240_steppers = [stepper for stepper in steppers if stepper[CONF_PLATFORM].startswith(CONF_TMC5240)]
    cg.add_define("TMC5240_NUM_COMPONENTS", len(tmc5240_steppers))
    return config

FINAL_VALIDATE_SCHEMA = final_validate_config
