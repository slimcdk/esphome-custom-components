import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.core import EsphomeError
from esphome.const import (
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_ID,
    CONF_STEP_PIN,
    CONF_DIR_PIN,
)

CODEOWNERS = ["@slimcdk"]

CONF_TMC2209 = "tmc2209"
CONF_TMC2209_ID = "tmc2209_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"

CONF_CLOCK_FREQUENCY = "clock_frequency"

CONF_OTTRIM = "ottrim"
CONF_VSENSE = "vsense"  # true lowers power dissipation in sense resistors
CONF_RSENSE = "rsense"  # sense resistors
CONF_INTERNAL_RSENSE = "internal_rsense"  # use rdson to sense

CONF_POLL_STATUS_INTERVAL = "poll_status_interval"

CONF_ON_EVENT = "on_event"  # TODO: rename?

CONF_INVERSE_DIRECTION = "inverse_direction"
CONF_VELOCITY = "velocity"
CONF_MICROSTEPS = "microsteps"  # CHOPCONF.mres
CONF_INTERPOLATION = "interpolation"  # CHOPCONF.intpol

# CONF_VACTUAL = "vactual"

CONF_RUN_CURRENT = "run_current"  # translates to IRUN
CONF_HOLD_CURRENT = "hold_current"  # translates to IHOLD
CONF_IRUN = "irun"
CONF_IHOLD = "ihold"
CONF_IHOLDDELAY = "iholddelay"
CONF_TPOWERDOWN = "tpowerdown"
CONF_ENABLE_SPREADCYCLE = "enable_spreadcycle"
CONF_ACTIVATION_LEVEL = "activation_level"

CONF_TCOOL_THRESHOLD = "tcool_threshold"

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

tmc2209_ns = cg.esphome_ns.namespace("tmc2209")
TMC2209API = tmc2209_ns.class_("TMC2209API", uart.UARTDevice)
TMC2209Component = tmc2209_ns.class_("TMC2209Component", TMC2209API, cg.Component)

OnAlertTrigger = tmc2209_ns.class_("OnAlertTrigger", automation.Trigger)
ConfigureAction = tmc2209_ns.class_("ConfigureAction", automation.Action)
CurrentsAction = tmc2209_ns.class_("CurrentsAction", automation.Action)
StallguardAction = tmc2209_ns.class_("StallguardAction", automation.Action)
CoolstepAction = tmc2209_ns.class_("CoolstepAction", automation.Action)

DriverEvent = tmc2209_ns.enum("DriverEvent")

DEVICE_SCHEMA = cv.Schema({cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209Component)})


TMC2209_BASE_CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Optional(CONF_ENN_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_STEP_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
            cv.Optional(CONF_RSENSE): cv.resistance,  # default is 170 mOhm
            cv.Optional(CONF_VSENSE): cv.boolean,  # default OTP
            cv.Optional(CONF_OTTRIM): cv.int_range(0, 3),  # default OTP
            cv.Optional(CONF_CLOCK_FREQUENCY, default=12_000_000): cv.All(
                cv.positive_int, cv.frequency
            ),
            cv.Optional(
                CONF_POLL_STATUS_INTERVAL, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_EVENT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnAlertTrigger),
                }
            ),
        },
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def register_tmc2209_base(var, config):

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    enn_pin = config.get(CONF_ENN_PIN, None)
    diag_pin = config.get(CONF_DIAG_PIN, None)
    index_pin = config.get(CONF_INDEX_PIN, None)
    dir_pin = config.get(CONF_DIR_PIN, None)
    step_pin = config.get(CONF_STEP_PIN, None)
    alert_triggers = config.get(CONF_ON_EVENT, [])

    if enn_pin is not None:
        cg.add(var.set_enn_pin(await cg.gpio_pin_expression(enn_pin)))

    if diag_pin is not None:
        cg.add(var.set_diag_pin(await cg.gpio_pin_expression(diag_pin)))

    if index_pin is not None:
        cg.add(var.set_index_pin(await cg.gpio_pin_expression(index_pin)))

    if step_pin is not None and dir_pin is not None:
        cg.add(var.set_step_pin(await cg.gpio_pin_expression(step_pin)))
        cg.add(var.set_dir_pin(await cg.gpio_pin_expression(dir_pin)))

    if (vsense := config.get(CONF_VSENSE, None)) is not None:
        cg.add_define("VSENSE", vsense)

    if (ottrim := config.get(CONF_OTTRIM, None)) is not None:
        cg.add_define("OTTRIM", ottrim)

    if len(alert_triggers) > 0:
        for conf in alert_triggers:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [(DriverEvent, "event")], conf)
        # cg.add_define("ENABLE_DRIVER_EVENT_EVENTS")

    cg.add_define("POLL_STATUS_INTERVAL", config[CONF_POLL_STATUS_INTERVAL])

    cg.add_build_flag("-std=c++17")
    cg.add_build_flag("-std=gnu++17")

    return var


def validate_tmc2209_base(config):
    return config
    has_index = CONF_INDEX_PIN in config
    has_step = CONF_STEP_PIN in config
    has_dir = CONF_DIR_PIN in config

    if not has_index and not has_step and not has_dir:
        raise EsphomeError(
            f"A control must be defined! Configure either {CONF_INDEX_PIN} or {CONF_STEP_PIN} and {CONF_DIR_PIN}"
        )

    if has_index and (has_step or has_dir):
        raise EsphomeError(
            f"{CONF_INDEX_PIN} and {CONF_STEP_PIN} or {CONF_DIR_PIN} must not be configured together."
        )

    if (has_step and not has_dir) or (not has_step and has_dir):
        raise EsphomeError(
            f"{CONF_STEP_PIN} and {CONF_DIR_PIN} must be configured together."
        )

    if CONF_VSENSE in config and CONF_RSENSE not in config:
        raise EsphomeError(
            f"Must configure {CONF_RSENSE} if {CONF_VSENSE} is configured."
        )


@automation.register_action(
    "tmc2209.configure",
    ConfigureAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_INVERSE_DIRECTION): cv.boolean,
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 1)
            ),
            cv.Optional(CONF_INTERPOLATION): cv.templatable(cv.boolean),
            cv.Optional(CONF_ENABLE_SPREADCYCLE): cv.boolean,
        }
    ),
)
def tmc2209_configure_to_code(config, action_id, template_arg, args):
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

    if (en_spreadcycle := config.get(CONF_ENABLE_SPREADCYCLE, None)) is not None:
        template_ = yield cg.templatable(en_spreadcycle, args, bool)
        cg.add(var.set_enable_spreadcycle(template_))

    yield var


@automation.register_action(
    "tmc2209.currents",
    CurrentsAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
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
def tmc2209_currents_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

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
    "tmc2209.coolstep",
    CoolstepAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_TCOOL_THRESHOLD): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
        }
    ),
)
def tmc2209_tcool_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (tcoolthrs := config.get(CONF_TCOOL_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(tcoolthrs, args, int)
        cg.add(var.set_tcool_threshold(template_))

    yield var


TMC2209_FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    CONF_TMC2209, require_rx=True, require_tx=True
)
