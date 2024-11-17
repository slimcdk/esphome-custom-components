from esphome.core import EsphomeError
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import stepper
from esphome.const import CONF_ADDRESS, CONF_ID

from .. import (
    CONF_TMC2300_ID,
    CONF_CLOCK_FREQUENCY,
    CONF_RSENSE,
    CONF_ANALOG_SCALE,
    CONF_DIAG_PIN,
    CONF_DIR_PIN,
    CONF_STEP_PIN,
    TMC2300Component,
    TMC2300_BASE_CONFIG_SCHEMA,
    TMC2300_FINAL_VALIDATE_SCHEMA,
    register_tmc2300_base,
    validate_tmc2300_base,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc2300"]

tmc2300_stepper_ns = cg.esphome_ns.namespace("tmc2300_stepper")
TMC2300Stepper = tmc2300_stepper_ns.class_(
    "TMC2300Stepper", TMC2300Component, stepper.Stepper
)


DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2300_ID): cv.use_id(TMC2300Stepper),
    }
)


def validate_control_method_(config):
    has_diag_pin = CONF_DIAG_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if not has_diag_pin and not has_stepdir_pins:
        raise cv.Invalid(
            f"Either {CONF_DIAG_PIN} and/or {CONF_STEP_PIN} and {CONF_DIR_PIN} must be configured"
        )
    return config


CONFIG_SCHEMA = cv.All(
    TMC2300_BASE_CONFIG_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC2300Stepper),
        }
    ).extend(stepper.STEPPER_SCHEMA),
    cv.has_none_or_all_keys(CONF_STEP_PIN, CONF_DIR_PIN),
    validate_control_method_,
    validate_tmc2300_base,
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

    await register_tmc2300_base(var, config)
    await stepper.register_stepper(var, config)

    has_diag_pin = CONF_DIAG_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if has_stepdir_pins:
        cg.add_define("PULSES_CONTROL")
    elif has_diag_pin:
        cg.add_define("SERIAL_CONTROL")
    else:
        raise EsphomeError("Could not determine control method!")


FINAL_VALIDATE_SCHEMA = TMC2300_FINAL_VALIDATE_SCHEMA
