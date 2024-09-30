import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import automation, pins
from esphome.core import EsphomeError
from esphome.components import uart, stepper
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_ID,
    CONF_PLATFORM,
    CONF_STEP_PIN,
    CONF_DIR_PIN,
    CONF_ENABLE_PIN,
)

CODEOWNERS = ["@slimcdk"]

CONF_STEPPER = "stepper"
CONF_TMC2300 = "tmc2300"
CONF_TMC2300_ID = "tmc2300_id"

CONF_ENABLE = "enable"
CONF_DISABLE = "disable"

CONF_DIAG_PIN = "diag_pin"
CONF_NSTDBY_PIN = "nstdby_pin"

CONF_CLOCK_FREQUENCY = "clock_frequency"

CONF_RSENSE = "rsense"  # sense resistors
CONF_POLL_STATUS_INTERVAL = "poll_status_interval"

CONF_ON_ALERT = "on_alert"

CONF_INVERSE_DIRECTION = "inverse_direction"
CONF_VELOCITY = "velocity"
CONF_MICROSTEPS = "microsteps"  # CHOPCONF.mres
CONF_INTERPOLATION = "interpolation"  # CHOPCONF.intpol

CONF_VACTUAL = "vactual"

CONF_COOLSTEP_TCOOL_THRESHOLD = "tcool_threshold"
CONF_STALLGUARD_THRESHOLD = "stallguard_threshold"

CONF_RUN_CURRENT = "run_current"  # translates to IRUN
CONF_HOLD_CURRENT = "hold_current"  # translates to IHOLD
CONF_IRUN = "irun"
CONF_IHOLD = "ihold"
CONF_IHOLDDELAY = "iholddelay"
CONF_TPOWERDOWN = "tpowerdown"

CONF_STANDSTILL_MODE = "standstill_mode"
STANDSTILL_MODE_NORMAL = "normal"
STANDSTILL_MODE_FREEWHEELING = "freewheeling"
STANDSTILL_MODE_COIL_SHORT_LS = "short_coil_ls"
STANDSTILL_MODE_COIL_SHORT_HS = "short_coil_hs"

STANDSTILL_MODES = {
    STANDSTILL_MODE_NORMAL: 0,
    STANDSTILL_MODE_FREEWHEELING: 1,
    STANDSTILL_MODE_COIL_SHORT_LS: 2,
    STANDSTILL_MODE_COIL_SHORT_HS: 3,
}

tmc2300_ns = cg.esphome_ns.namespace("tmc2300")
TMC2300 = tmc2300_ns.class_("TMC2300", cg.Component, uart.UARTDevice, stepper.Stepper)

OnAlertTrigger = tmc2300_ns.class_("OnAlertTrigger", automation.Trigger)
ConfigureAction = tmc2300_ns.class_("ConfigureAction", automation.Action)
ActivationAction = tmc2300_ns.class_("ActivationAction", automation.Action)

DriverEvent = tmc2300_ns.enum("DriverEvent")


DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2300_ID): cv.use_id(TMC2300),
    }
)


def validate_(config):

    has_diag = CONF_DIAG_PIN in config
    has_step = CONF_STEP_PIN in config
    has_dir = CONF_DIR_PIN in config

    if not has_diag and not has_step and not has_dir:
        raise EsphomeError(
            f"Configure minimum {CONF_DIAG_PIN}, {CONF_STEP_PIN} and {CONF_DIR_PIN} or all three."
        )

    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC2300),
            cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
            cv.Optional(CONF_CLOCK_FREQUENCY, default=12_000_000): cv.frequency,
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_NSTDBY_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_STEP_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_RSENSE): cv.resistance,
            cv.Optional(
                CONF_POLL_STATUS_INTERVAL, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_ALERT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnAlertTrigger),
                }
            ),
        },
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(stepper.STEPPER_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    cv.has_none_or_all_keys(CONF_STEP_PIN, CONF_DIR_PIN),
    validate_,
)


async def to_code(config):

    var = cg.new_Pvariable(
        config[CONF_ID], config[CONF_ADDRESS], config[CONF_CLOCK_FREQUENCY]
    )
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await stepper.register_stepper(var, config)

    en_pin = config.get(CONF_ENABLE_PIN, None)
    nstdby_pin = config.get(CONF_NSTDBY_PIN, None)
    diag_pin = config.get(CONF_DIAG_PIN, None)
    step_pin = config.get(CONF_STEP_PIN, None)
    dir_pin = config.get(CONF_DIR_PIN, None)
    alert_triggers = config.get(CONF_ON_ALERT, [])

    if en_pin is not None:
        cg.add(var.set_en_pin(await cg.gpio_pin_expression(en_pin)))

    if nstdby_pin is not None:
        cg.add(var.set_nstdby_pin(await cg.gpio_pin_expression(nstdby_pin)))

    pulse_control = step_pin is not None and dir_pin is not None
    if pulse_control:
        cg.add(var.set_step_pin(await cg.gpio_pin_expression(step_pin)))
        cg.add(var.set_dir_pin(await cg.gpio_pin_expression(dir_pin)))
        cg.add_define("USE_PULSE_CONTROL")

    if diag_pin is not None:
        cg.add(var.set_diag_pin(await cg.gpio_pin_expression(diag_pin)))
        cg.add_define("USE_DIAG_PIN" if pulse_control else "USE_UART_CONTROL")

    cg.add_define("RSENSE", config.get(CONF_RSENSE, 0.170))

    if len(alert_triggers) > 0:
        cg.add_define("ENABLE_DRIVER_ALERT_EVENTS")
        for conf in alert_triggers:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [(DriverEvent, "alert")], conf)

    cg.add_define("POLL_STATUS_INTERVAL", config[CONF_POLL_STATUS_INTERVAL])

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.10.3")
    cg.add_build_flag("-std=c++17")
    cg.add_build_flag("-std=gnu++17")


@automation.register_action(
    "tmc2300.configure",
    ConfigureAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2300),
            cv.Optional(CONF_INVERSE_DIRECTION): cv.boolean,
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 1)
            ),
            cv.Optional(CONF_INTERPOLATION): cv.templatable(cv.boolean),
            cv.Optional(CONF_COOLSTEP_TCOOL_THRESHOLD): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
            cv.Optional(CONF_STALLGUARD_THRESHOLD): cv.templatable(
                cv.int_range(min=0, max=2**8)
            ),
            cv.Exclusive(CONF_RUN_CURRENT, "run_current"): cv.templatable(
                cv.All(cv.current, cv.positive_not_null_float)
            ),
            cv.Exclusive(CONF_IRUN, "run_current"): cv.templatable(
                cv.int_range(min=0, max=31)
            ),
            cv.Exclusive(CONF_HOLD_CURRENT, "hold_current"): cv.templatable(
                cv.All(cv.current, cv.positive_float)
            ),
            cv.Exclusive(CONF_IHOLD, "hold_current"): cv.templatable(
                cv.int_range(min=0, max=31)
            ),
            cv.Optional(CONF_STANDSTILL_MODE): cv.enum(STANDSTILL_MODES),
            cv.Optional(CONF_IHOLDDELAY): cv.int_range(0, 15),
            cv.Optional(CONF_TPOWERDOWN): cv.int_range(0, 255),
        }
    ),
)
def tmc2300_configure_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (inverse_direction := config.get(CONF_INVERSE_DIRECTION, None)) is not None:
        template_ = yield cg.templatable(inverse_direction, args, bool)
        cg.add(var.set_inverse_direction(template_))

    if (microsteps := config.get(CONF_MICROSTEPS, None)) is not None:
        template_ = yield cg.templatable(microsteps, args, int)
        cg.add(var.set_microsteps(template_))

    if (interpolation := config.get(CONF_INTERPOLATION, None)) is not None:
        template_ = yield cg.templatable(interpolation, args, bool)
        cg.add(var.set_microstep_interpolation(template_))

    if (tcoolthrs := config.get(CONF_COOLSTEP_TCOOL_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(tcoolthrs, args, int)
        cg.add(var.set_coolstep_tcoolthrs(template_))

    if (sgthrs := config.get(CONF_STALLGUARD_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(sgthrs, args, int)
        cg.add(var.set_stallguard_sgthrs(template_))

    if (standstill_mode := config.get(CONF_STANDSTILL_MODE, None)) is not None:
        template_ = yield cg.templatable(STANDSTILL_MODES[standstill_mode], args, float)
        cg.add(var.set_standstill_mode(template_))

    if (run_current := config.get(CONF_RUN_CURRENT, None)) is not None:
        template_ = yield cg.templatable(run_current, args, cv.current)
        cg.add(var.set_run_current(template_))

    if (irun := config.get(CONF_IRUN, None)) is not None:
        template_ = yield cg.templatable(irun, args, cv.int_range(0, 31))
        cg.add(var.set_irun(template_))

    if (hold_current := config.get(CONF_HOLD_CURRENT, None)) is not None:
        template_ = yield cg.templatable(hold_current, args, cv.current)
        cg.add(var.set_hold_current(template_))

    if (ihold := config.get(CONF_IHOLD, None)) is not None:
        template_ = yield cg.templatable(ihold, args, cv.int_range(0, 31))
        cg.add(var.set_ihold(template_))

    if (iholddelay := config.get(CONF_IHOLDDELAY, None)) is not None:
        template_ = yield cg.templatable(iholddelay, args, float)
        cg.add(var.set_iholddelay(template_))

    if (tpowerdown := config.get(CONF_TPOWERDOWN, None)) is not None:
        template_ = yield cg.templatable(tpowerdown, args, float)
        cg.add(var.set_tpowerdown(template_))

    yield var


@automation.register_action(
    "tmc2300.enable",
    ActivationAction,
    maybe_simple_id({cv.GenerateID(): cv.use_id(TMC2300)}),
)
def tmc2300_enable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(True))
    return var


@automation.register_action(
    "tmc2300.disable",
    ActivationAction,
    maybe_simple_id({cv.GenerateID(): cv.use_id(TMC2300)}),
)
def tmc2300_disable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(False))
    return var


def final_validate_config(config):

    uart.final_validate_device_schema(CONF_TMC2300, require_rx=True, require_tx=True)(
        config
    )

    steppers = fv.full_config.get()[CONF_STEPPER]
    tmc2300s = [
        stepper for stepper in steppers if stepper[CONF_PLATFORM] == CONF_TMC2300
    ]

    cg.add_define("TMC2300_NUM_COMPONENTS", len(tmc2300s))
    cg.add_define("TMC2300_ENABLE_TMC_CACHE", len(tmc2300s))
    cg.add_define("TMC2300_CACHE", True)
    cg.add_define("TMC_API_EXTERNAL_CRC_TABLE", False)


FINAL_VALIDATE_SCHEMA = final_validate_config
