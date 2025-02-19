from esphome.const import CONF_ID, CONF_VARIANT
from esphome.core import EsphomeError
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import stepper
from .. import (
    CONF_TMC22XX_ID,
    CONF_INDEX_PIN,
    CONF_DIR_PIN,
    CONF_STEP_PIN,
    tmc22xx_ns,
    TMC22XXComponent,
    register_tmc22xx_base,
    _build_typed_schema,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc22xx_hub", "tmc22xx"]

TMC22XXStepper = tmc22xx_ns.class_("TMC22XXStepper", TMC22XXComponent, stepper.Stepper)
ControlMethod = tmc22xx_ns.enum("ControlMethod")

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XXStepper),
    }
)


def _validate_control_method(config):
    has_index_pin = CONF_INDEX_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if not has_index_pin and not has_stepdir_pins:
        raise cv.Invalid(
            f"Either {CONF_INDEX_PIN} and/or {CONF_STEP_PIN} and {CONF_DIR_PIN} must be configured"
        )
    return config


CONFIG_SCHEMA = cv.All(
    _build_typed_schema(TMC22XXStepper, extend=stepper.STEPPER_SCHEMA),
    cv.has_none_or_all_keys(CONF_STEP_PIN, CONF_DIR_PIN),
    _validate_control_method,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await register_tmc22xx_base(var, config)
    await stepper.register_stepper(var, config)

    has_index_pin = CONF_INDEX_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if has_index_pin:
        cg.add(var.set_control_method(ControlMethod.SERIAL_CONTROL))
    elif has_stepdir_pins:
        cg.add(var.set_control_method(ControlMethod.PULSES_CONTROL))
    else:
        raise EsphomeError("Could not determine control method!")
