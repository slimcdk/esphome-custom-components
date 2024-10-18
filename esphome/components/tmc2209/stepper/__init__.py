import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import automation, pins
from esphome.core import EsphomeError
from esphome.components import uart, stepper
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    CONF_THRESHOLD,
)


from .. import (
    CONF_TMC2209_ID,
    tmc2209_ns,
    TMC2209Component,
    TMC2209_BASE_CONFIG_SCHEMA,
    CONF_CLOCK_FREQUENCY,
    CONF_RSENSE,
    CONF_ACTIVATION_LEVEL,
    StallguardAction,
    register_tmc2209_base,
    validate_tmc2209_base,
    TMC2209_FINAL_VALIDATE_SCHEMA,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc2209"]


TMC2209Stepper = tmc2209_ns.class_("TMC2209Stepper", TMC2209Component, stepper.Stepper)
ActivationAction = tmc2209_ns.class_("ActivationAction", automation.Action)

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209Stepper),
    }
)


CONFIG_SCHEMA = cv.All(
    TMC2209_BASE_CONFIG_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(TMC2209Stepper),
        }
    ).extend(stepper.STEPPER_SCHEMA),
    validate_tmc2209_base,
)


async def to_code(config):

    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_ADDRESS],
        config[CONF_CLOCK_FREQUENCY],
        not config.get(CONF_RSENSE),  # internal rsense
        config.get(CONF_RSENSE, 0.170),  # rsense value
    )

    await register_tmc2209_base(var, config)
    await stepper.register_stepper(var, config)

    cg.add_define("USE_UART_CONTROL")
    cg.add_build_flag("-std=c++17")
    cg.add_build_flag("-std=gnu++17")


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
            cv.Optional(CONF_ACTIVATION_LEVEL, default=0.5): cv.percentage,
        }
    ),
)
def tmc2209_stallguard_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (sgthrs := config.get(CONF_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(sgthrs, args, int)
        cg.add(var.set_stallguard_threshold(template_))

    sal = config[CONF_ACTIVATION_LEVEL]
    sal_template_ = yield cg.templatable(sal, args, cv.percentage)
    cg.add(var.set_stall_detection_activation_level(sal_template_))

    yield var


FINAL_VALIDATE_SCHEMA = TMC2209_FINAL_VALIDATE_SCHEMA
