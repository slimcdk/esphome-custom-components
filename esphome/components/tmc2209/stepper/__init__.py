from esphome.const import CONF_ADDRESS, CONF_ID
from esphome.core import EsphomeError
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import stepper
from .. import (
    CONF_TMC2209_ID,
    CONF_CLOCK_FREQUENCY,
    CONF_RSENSE,
    CONF_ANALOG_SCALE,
    CONF_INDEX_PIN,
    CONF_DIR_PIN,
    CONF_STEP_PIN,
    tmc2209_ns,
    TMC2209Component,
    TMC2209_BASE_CONFIG_SCHEMA,
    register_tmc2209_base,
    validate_tmc2209_base,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc2209_hub", "tmc2209"]

TMC2209Stepper = tmc2209_ns.class_("TMC2209Stepper", TMC2209Component, stepper.Stepper)
ControlMethod = tmc2209_ns.enum("ControlMethod")


DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209Stepper),
    }
)


def validate_control_method_(config):
    has_index_pin = CONF_INDEX_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if not has_index_pin and not has_stepdir_pins:
        raise cv.Invalid(
            f"Either {CONF_INDEX_PIN} and/or {CONF_STEP_PIN} and {CONF_DIR_PIN} must be configured"
        )
    return config


CONFIG_SCHEMA = cv.All(
    TMC2209_BASE_CONFIG_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC2209Stepper),
        }
    ).extend(stepper.STEPPER_SCHEMA),
    cv.has_none_or_all_keys(CONF_STEP_PIN, CONF_DIR_PIN),
    validate_control_method_,
    validate_tmc2209_base,
)


async def to_code(config):

    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_ADDRESS],
        config[CONF_CLOCK_FREQUENCY],
        CONF_RSENSE not in config,  # internal rsense
        config[CONF_RSENSE],  # rsense value
        config[CONF_ANALOG_SCALE],  # VREF is connected
    )

    await register_tmc2209_base(var, config)
    await stepper.register_stepper(var, config)

    has_index_pin = CONF_INDEX_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if has_stepdir_pins:
        cg.add(var.set_control_method(ControlMethod.PULSES))
    elif has_index_pin:
        cg.add(var.set_control_method(ControlMethod.SERIAL))
    else:
        raise EsphomeError("Could not determine control method!")
