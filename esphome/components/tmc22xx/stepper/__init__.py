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
    CONF_ON_STALL,
    tmc22xx_ns,
    TMC22XXComponent,
    TMC2202Component,
    TMC2208Component,
    TMC2209Component,
    TMC2226Component,
    TMC2224Component,
    TMC2225Component,
    TMC22XX_BASE_CONFIG_SCHEMA,
    register_tmc22xx_base,
    VARIANT_TMC2202,
    VARIANT_TMC2208,
    VARIANT_TMC2209,
    VARIANT_TMC2226,
    VARIANT_TMC2224,
    VARIANT_TMC2225,
    TMC22XX_STALL_TRIGGER_CONFIG_SCHEMA,
    TMC22XX_CONFIG_SCHEMA,
    _on_stall_invalid,
    _build_typed_schema,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc22xx_hub", "tmc22xx"]

TMC22XXStepper = tmc22xx_ns.class_("TMC22XXStepper", TMC22XXComponent, stepper.Stepper)
TMC2202Stepper = tmc22xx_ns.class_("TMC2202Stepper", TMC2202Component, stepper.Stepper)
TMC2208Stepper = tmc22xx_ns.class_("TMC2208Stepper", TMC2208Component, stepper.Stepper)
TMC2209Stepper = tmc22xx_ns.class_("TMC2209Stepper", TMC2209Component, stepper.Stepper)
TMC2224Stepper = tmc22xx_ns.class_("TMC2224Stepper", TMC2224Component, stepper.Stepper)
TMC2225Stepper = tmc22xx_ns.class_("TMC2225Stepper", TMC2225Component, stepper.Stepper)
TMC2226Stepper = tmc22xx_ns.class_("TMC2226Stepper", TMC2226Component, stepper.Stepper)

ControlMethod = tmc22xx_ns.enum("ControlMethod")

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XXStepper),
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
    _build_typed_schema(
        tmc2202=TMC2202Stepper,
        tmc2208=TMC2208Stepper,
        tmc2209=TMC2209Stepper,
        tmc2224=TMC2224Stepper,
        tmc2225=TMC2225Stepper,
        tmc2226=TMC2226Stepper,
        extend=stepper.STEPPER_SCHEMA,
    ),
    cv.has_none_or_all_keys(CONF_STEP_PIN, CONF_DIR_PIN),
    validate_control_method_,
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
