import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import stepper
from esphome.automation import maybe_simple_id
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_TRIGGER_ID, CONF_THRESHOLD
from esphome.core import EsphomeError

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
    TMC2209_FINAL_VALIDATE_SCHEMA,
    register_tmc2209_base,
    validate_tmc2209_base,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc2209"]

OPT_SERIAL_CONTROL = "serial"
OPT_PULSES_CONTROL = "pulses"
OPT_HYBRID_CONTROL = "hybrid"

CONF_OVERRIDE_CONTROL_METHOD = "override_control_method"
CONF_ACTIVATION_LEVEL = "activation_level"

CONF_ON_STALL = "on_stall"


TMC2209Stepper = tmc2209_ns.class_("TMC2209Stepper", TMC2209Component, stepper.Stepper)
ActivationAction = tmc2209_ns.class_("ActivationAction", automation.Action)
StallguardAction = tmc2209_ns.class_("StallguardAction", automation.Action)

OnStallTrigger = tmc2209_ns.class_("OnStallTrigger", automation.Trigger)

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209Stepper),
    }
)


def get_inferred_control_method_(config):
    has_index_pin = CONF_INDEX_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config

    if has_index_pin and has_stepdir_pins:
        return OPT_HYBRID_CONTROL
    if has_stepdir_pins:
        return OPT_PULSES_CONTROL
    if has_index_pin:
        return OPT_SERIAL_CONTROL
    return None


def get_control_method_(config):
    cm = get_inferred_control_method_(config)
    ocm = config.get(CONF_OVERRIDE_CONTROL_METHOD, None)
    return ocm if ocm is not None else cm


def control_method_as_cpp_definition_(method):
    lookup = {
        OPT_SERIAL_CONTROL: "SERIAL_CONTROL",
        OPT_PULSES_CONTROL: "PULSES_CONTROL",
        OPT_HYBRID_CONTROL: "HYBRID_CONTROL",
    }
    return lookup.get(method, None)


def validate_control_method_(config):

    has_index_pin = CONF_INDEX_PIN in config
    has_stepdir_pins = CONF_STEP_PIN in config and CONF_DIR_PIN in config
    method = get_control_method_(config)

    if method is None or not (has_index_pin or has_stepdir_pins):
        raise EsphomeError(
            f"{CONF_INDEX_PIN} or {CONF_STEP_PIN} and {CONF_DIR_PIN} must be configured",
        )

    if method == OPT_SERIAL_CONTROL and not has_index_pin:
        raise EsphomeError(f"{CONF_INDEX_PIN} is required for 'serial' control")

    if method == OPT_PULSES_CONTROL and not has_stepdir_pins:
        raise EsphomeError(
            f"{CONF_STEP_PIN} and {CONF_DIR_PIN} is required for 'pulses' control",
        )

    if method == OPT_HYBRID_CONTROL and not (has_stepdir_pins and has_index_pin):
        raise EsphomeError(
            f"{CONF_INDEX_PIN}, {CONF_STEP_PIN} and {CONF_DIR_PIN} is required for 'hybrid' control",
        )

    config[CONF_OVERRIDE_CONTROL_METHOD] = method
    return config


CONFIG_SCHEMA = cv.All(
    TMC2209_BASE_CONFIG_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC2209Stepper),
            cv.Optional(CONF_OVERRIDE_CONTROL_METHOD): cv.one_of(
                OPT_SERIAL_CONTROL, OPT_PULSES_CONTROL, OPT_HYBRID_CONTROL
            ),
            cv.Optional(CONF_ON_STALL): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnStallTrigger),
                }
            ),
        }
    ).extend(stepper.STEPPER_SCHEMA),
    validate_control_method_,
    validate_tmc2209_base,
)


async def to_code(config):

    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_ADDRESS],
        config[CONF_CLOCK_FREQUENCY],
        CONF_RSENSE not in config,  # internal rsense
        config.get(CONF_RSENSE, 0.170),  # rsense value
        config[CONF_ANALOG_SCALE],  # VREF is connected
    )

    await register_tmc2209_base(var, config)
    await stepper.register_stepper(var, config)

    method = config.get(CONF_OVERRIDE_CONTROL_METHOD)
    cg.add_define(control_method_as_cpp_definition_(method))

    for conf in config.get(CONF_ON_STALL, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


@automation.register_action(
    "tmc2209.enable",
    ActivationAction,
    maybe_simple_id({cv.GenerateID(): cv.use_id(TMC2209Component)}),
)
def tmc2209_enable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(True))
    return var


@automation.register_action(
    "tmc2209.disable",
    ActivationAction,
    maybe_simple_id({cv.GenerateID(): cv.use_id(TMC2209Component)}),
)
def tmc2209_disable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(False))
    return var


@automation.register_action(
    "tmc2209.stallguard",
    StallguardAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_THRESHOLD): cv.templatable(cv.int_range(min=0, max=2**8)),
            cv.Optional(CONF_ACTIVATION_LEVEL, default=0.1): cv.percentage,
        }
    ),
)
def tmc2209_stallguard_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (sgthrs := config.get(CONF_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(sgthrs, args, int)
        cg.add(var.set_stallguard_threshold(template_))

    if (level := config.get(CONF_ACTIVATION_LEVEL, None)) is not None:
        cg.add(var.set_activation_level(level))

    yield var


FINAL_VALIDATE_SCHEMA = TMC2209_FINAL_VALIDATE_SCHEMA
