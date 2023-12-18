from esphome import automation, pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.components import uart, stepper

from esphome.const import CONF_TRIGGER_ID, CONF_ADDRESS, CONF_ID, CONF_PLATFORM

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["uart"]
CONF_STEPPER = "stepper"
CONF_TMC2209 = "tmc2209"
CONF_TMC2209_ID = "tmc2209_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"

CONF_INVERSE_DIRECTION = "inverse_direction"

CONF_VELOCITY = "velocity"

CONF_VACTUAL = "vactual"
CONF_MICROSTEPS = "microsteps"
CONF_COOLSTEP_TCOOLTHRS = "coolstep_tcoolthrs"
CONF_STALLGUARD_SGTHRS = "stallguard_sgthrs"

CONF_RMS_CURRENT = "rms_current"
CONF_RMS_CURRENT_HOLD_SCALE = "rms_current_hold_scale"

CONF_HOLD_CURRENT_DELAY = "hold_current_delay"
CONF_POWER_DOWN_DELAY = "power_down_delay"
CONF_TSTEP = "tstep"
CONF_TPWMTHRS = "tpwmthrs"
CONF_RSENSE = "rsense"
CONF_INTERNAL_RSENSE = "internal_rsense"

CONF_OSCILLATOR_FREQUENCY = "oscillator_freq"

CONF_COOLCONF_SEIMIN = "coolstep_seimin"
CONF_COOLCONF_SEDN1 = "coolstep_sedn1"
CONF_COOLCONF_SEDN0 = "coolstep_sedn0"
CONF_COOLCONF_SEDN1 = "coolstep_sedn1"
CONF_COOLCONF_SEMAX3 = "coolstep_semax3"
CONF_COOLCONF_SEMAX2 = "coolstep_semax2"
CONF_COOLCONF_SEMAX1 = "coolstep_semax1"
CONF_COOLCONF_SEMAX0 = "coolstep_semax0"
CONF_COOLCONF_SEUP1 = "coolstep_seup1"
CONF_COOLCONF_SEUP0 = "coolstep_seup0"
CONF_COOLCONF_SEMIN3 = "coolstep_semin3"
CONF_COOLCONF_SEMIN2 = "coolstep_semin2"
CONF_COOLCONF_SEMIN1 = "coolstep_semin1"
CONF_COOLCONF_SEMIN0 = "coolstep_semin0"

CONF_ON_STALL = "on_stall"

tmc_ns = cg.esphome_ns.namespace("tmc")
TMC2209Stepper = tmc_ns.class_("TMC2209Stepper", cg.Component, stepper.Stepper, uart.UARTDevice)

TMC2209StepperConfigureAction = tmc_ns.class_("TMC2209StepperConfigureAction", automation.Action)
TMC2209StepperStopAction = tmc_ns.class_("TMC2209StepperStopAction", automation.Action)
TMC2209StepperOnStallTrigger = tmc_ns.class_("TMC2209StepperOnStallTrigger", automation.Trigger)


CONFIG_SCHEMA = cv.All(
    stepper.STEPPER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TMC2209Stepper),
            cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0x00): cv.uint8_t,
            cv.Optional(CONF_RSENSE): cv.resistance,
            # cv.Optional(CONF_INTERNAL_RSENSE, default=True): cv.boolean,
            cv.Optional(CONF_OSCILLATOR_FREQUENCY, default=12_000_000): cv.frequency,
            cv.Optional(CONF_ON_STALL): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        TMC2209StepperOnStallTrigger
                    ),
                }
            ),
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_ADDRESS])
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_enn_pin(await cg.gpio_pin_expression(config[CONF_ENN_PIN])))
    cg.add(var.set_index_pin(await cg.gpio_pin_expression(config[CONF_INDEX_PIN])))

    if CONF_DIAG_PIN in config:
        cg.add_define("USE_DIAG_PIN")
        cg.add(var.set_diag_pin(await cg.gpio_pin_expression(config[CONF_DIAG_PIN])))

    cg.add(var.set_oscillator_frequency(config[CONF_OSCILLATOR_FREQUENCY]))

    cg.add(var.set_rsense(
        config[CONF_RSENSE] if CONF_RSENSE in config else 0,
        CONF_RSENSE not in config   # If not set, use internal vref sensing
    ))

    for conf in config.get(CONF_ON_STALL, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.5.2")


def final_validate_config(config):
    steppers = fv.full_config.get()[CONF_STEPPER]
    tmc2209_steppers = [stepper for stepper in steppers if stepper[CONF_PLATFORM] == CONF_TMC2209]
    cg.add_define("TMC2209_NUM_COMPONENTS", len(tmc2209_steppers))
    return config

FINAL_VALIDATE_SCHEMA = final_validate_config

@automation.register_action(
    "tmc2209.configure",
    TMC2209StepperConfigureAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TMC2209Stepper),
            cv.Optional(CONF_INVERSE_DIRECTION): cv.boolean,
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 0)
            ),
            cv.Optional(CONF_RMS_CURRENT): cv.templatable(cv.All(
                cv.current,
                cv.positive_float
            )),
            cv.Optional(CONF_RMS_CURRENT_HOLD_SCALE): cv.templatable(cv.percentage),
            cv.Optional(CONF_COOLSTEP_TCOOLTHRS): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
            cv.Optional(CONF_STALLGUARD_SGTHRS): cv.templatable(
                cv.int_range(min=0, max=2**8, max_included=True)
            ),
            # cv.Optional(CONF_HOLD_CURRENT_DELAY): cv.templatable(cv.All(
            #     cv.int_range(min=0, max=2**4, max_included=False),
            #     cv.time_period_in_milliseconds_
            # )),
            # cv.Optional(CONF_POWER_DOWN_DELAY): cv.templatable(cv.All(
            #     cv.int_range(min=0, max=2**8, max_included=False),
            #     cv.time_period_in_milliseconds_
            # )),
        }
    ),
)
def tmc2209_configure_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if CONF_INVERSE_DIRECTION in config:
        template_ = yield cg.templatable(config[CONF_INVERSE_DIRECTION], args, bool)
        cg.add(var.set_inverse_direction(template_))

    if CONF_RMS_CURRENT in config:
        template_ = yield cg.templatable(config[CONF_RMS_CURRENT], args, float)
        cg.add(var.set_rms_current(template_))

    if CONF_RMS_CURRENT_HOLD_SCALE in config:
        template_ = yield cg.templatable(config[CONF_RMS_CURRENT_HOLD_SCALE], args, float)
        cg.add(var.set_rms_current_hold_scale(template_))

    if CONF_HOLD_CURRENT_DELAY in config:
        template_ = yield cg.templatable(config[CONF_HOLD_CURRENT_DELAY], args, int)
        cg.add(var.ihold_irun_ihold_delay(template_))

    if CONF_MICROSTEPS in config:
        template_ = yield cg.templatable(config[CONF_MICROSTEPS], args, int)
        cg.add(var.set_microsteps(template_))

    if CONF_COOLSTEP_TCOOLTHRS in config:
        template_ = yield cg.templatable(config[CONF_COOLSTEP_TCOOLTHRS], args, int)
        cg.add(var.set_coolstep_tcoolthrs(template_))

    if CONF_STALLGUARD_SGTHRS in config:
        template_ = yield cg.templatable(config[CONF_STALLGUARD_SGTHRS], args, int)
        cg.add(var.set_stallguard_sgthrs(template_))
    yield var


TMC2209_ACTION_SCHEMA = automation.maybe_simple_id({
    cv.Required(CONF_ID): cv.use_id(TMC2209Stepper)
})

@automation.register_action("tmc2209.stop", TMC2209StepperStopAction, TMC2209_ACTION_SCHEMA)
def tmc2209_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    yield var
