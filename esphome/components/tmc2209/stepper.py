from esphome import automation, pins
from esphome.components import uart, stepper, esp32

from esphome.core import CORE, coroutine_with_priority
import esphome.config_validation as cv
import esphome.codegen as cg

from esphome.const import CONF_TRIGGER_ID, CONF_ADDRESS, CONF_ID

CODEOWNERS = ["@slimcdk"]


DEPENDENCIES = ["uart"]

CONF_TMC2209_ID = "tmc2209_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"
CONF_DIR_PIN = "dir_pin"
CONF_STEP_PIN = "step_pin"
CONF_STEP_FEEDBACK_PIN = "step_feedback_pin"



CONF_INVERSE_DIRECTION = "inverse_direction"

CONF_VELOCITY = "velocity"

CONF_VACTUAL = "vactual"
CONF_MICROSTEPS = "microsteps"
CONF_COOLSTEP_TCOOLTHRS = "coolstep_tcoolthrs"
CONF_STALLGUARD_SGTHRS = "stallguard_sgthrs"

CONF_RUN_CURRENT = "run_current"
CONF_HOLD_CURRENT = "hold_current"
CONF_HOLD_CURRENT_DELAY = "hold_current_delay"
CONF_POWER_DOWN_DELAY = "power_down_delay"
CONF_TSTEP = "tstep"
CONF_TPWMTHRS = "tpwmthrs"
CONF_RSENSE = "rsense"

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

CONF_ON_FAULT_SIGNAL = "on_fault_signal"

tmc_ns = cg.esphome_ns.namespace("tmc")
TMC2209Stepper = tmc_ns.class_("TMC2209Stepper", cg.Component, stepper.Stepper, uart.UARTDevice)

TMC2209StepperFaultSignalTrigger = tmc_ns.class_("TMC2209StepperFaultSignalTrigger", automation.Trigger)
TMC2209StepperConfigureAction = tmc_ns.class_("TMC2209StepperConfigureAction", automation.Action)
TMC2209StepperVelocityAction = tmc_ns.class_("TMC2209StepperVelocityAction", automation.Action)


CONFIG_SCHEMA = (
    stepper.STEPPER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TMC2209Stepper),
            cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0): cv.uint8_t,
            cv.Optional(CONF_RSENSE): cv.resistance,
            cv.Optional(CONF_ON_FAULT_SIGNAL): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        TMC2209StepperFaultSignalTrigger
                    ),
                }
            ),
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

    cg.add(var.set_enn_pin(await cg.gpio_pin_expression(config[CONF_ENN_PIN])))
    cg.add(var.set_diag_pin(await cg.gpio_pin_expression(config[CONF_DIAG_PIN])))
    cg.add(var.set_index_pin(await cg.gpio_pin_expression(config[CONF_INDEX_PIN])))

    cg.add(var.set_address(config[CONF_ADDRESS]))

    for conf in config.get(CONF_ON_FAULT_SIGNAL, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.5.2")



TMC2209_STEPPER_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209Stepper),
})


@automation.register_action(
    "tmc2209.configure",
    TMC2209StepperConfigureAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TMC2209Stepper),
            # Velocity
            cv.Optional(CONF_INVERSE_DIRECTION): cv.templatable(cv.boolean),
            cv.Optional(CONF_HOLD_CURRENT): cv.templatable(
                cv.int_range(min=0, max=2**5, max_included=False),
            ),
            cv.Optional(CONF_RUN_CURRENT, default=31): cv.templatable(
                cv.int_range(min=0, max=2**5, max_included=False),
            ),
            cv.Optional(CONF_HOLD_CURRENT_DELAY): cv.templatable(
                cv.int_range(min=0, max=2**4, max_included=False),
            ),
            cv.Optional(CONF_POWER_DOWN_DELAY): cv.templatable(
                cv.int_range(
                    min=0, max=2**8, max_included=False
                ),  # TODO: input value in time / duration format
            ),
            # Microstepping
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 0)
            ),
            cv.Optional(CONF_COOLSTEP_TCOOLTHRS): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
            cv.Optional(CONF_STALLGUARD_SGTHRS): cv.templatable(
                cv.int_range(min=0, max=2**8, max_included=True)
            ),
            cv.Optional(CONF_TSTEP): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),

            # CoolStep configuration
            cv.Optional(CONF_COOLCONF_SEIMIN): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEDN1): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEDN0): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEDN1): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMAX3): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMAX2): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMAX1): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMAX0): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEUP1): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEUP0): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMIN3): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMIN2): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMIN1): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLCONF_SEMIN0): cv.templatable(cv.boolean),
        }
    ),
)
def tmc2209_configure_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    if CONF_INVERSE_DIRECTION in config:
        template_ = yield cg.templatable(config[CONF_INVERSE_DIRECTION], args, bool)
        cg.add(var.set_inverse_direction(template_))

    if CONF_HOLD_CURRENT in config:
        template_ = yield cg.templatable(config[CONF_HOLD_CURRENT], args, int)  # float)
        cg.add(var.set_hold_current(template_))

    if CONF_RUN_CURRENT in config:
        template_ = yield cg.templatable(config[CONF_RUN_CURRENT], args, int)  # float)
        cg.add(var.set_run_current(template_))

    if CONF_HOLD_CURRENT_DELAY in config:
        template_ = yield cg.templatable(
            config[CONF_HOLD_CURRENT_DELAY], args, int
        )  # float)
        cg.add(var.set_hold_current_delay(template_))

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



@automation.register_action(
    "tmc2209.set_velocity",
    TMC2209StepperVelocityAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TMC2209Stepper),
            cv.Required(CONF_VELOCITY): cv.templatable(
                cv.int_range(min=-8388608, max=8388607),
            ),

        }
    ),
)
def tmc2209_set_velocity_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    template_ = yield cg.templatable(config[CONF_VELOCITY], args, int)
    cg.add(var.set_velocity(template_))

    yield var

