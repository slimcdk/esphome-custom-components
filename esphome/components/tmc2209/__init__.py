import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_ID,
    CONF_STEP_PIN,
    CONF_DIR_PIN,
    CONF_DIRECTION,
    CONF_THRESHOLD,
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
CONF_ANALOG_SCALE = "analog_scaling"
CONF_DRIVER_POLL_INTERVAL = "status_poll_interval"
CONF_ON_DRIVER_STATUS = "on_status"
CONF_ON_STALL = "on_stall"

OPT_CLOCKWISE = "clockwise"
OPT_CW = "cw"
OPT_COUNTERCLOCKWISE = "counterclockwise"
OPT_CCW = "ccw"
OPT_STANDSTILL_MODE_NORMAL = "normal"
OPT_STANDSTILL_MODE_FREEWHEELING = "freewheeling"
OPT_STANDSTILL_MODE_COIL_SHORT_LS = "short_coil_ls"
OPT_STANDSTILL_MODE_COIL_SHORT_HS = "short_coil_hs"

STANDSTILL_MODES = {
    OPT_STANDSTILL_MODE_NORMAL: 0,
    OPT_STANDSTILL_MODE_FREEWHEELING: 1,
    OPT_STANDSTILL_MODE_COIL_SHORT_LS: 2,
    OPT_STANDSTILL_MODE_COIL_SHORT_HS: 3,
}

CONF_VELOCITY = "velocity"
CONF_MICROSTEPS = "microsteps"  # CHOPCONF.mres
CONF_INTERPOLATION = "interpolation"  # CHOPCONF.intpol
CONF_RUN_CURRENT = "run_current"  # translates to IRUN
CONF_HOLD_CURRENT = "hold_current"  # translates to IHOLD
CONF_IRUN = "irun"
CONF_IHOLD = "ihold"
CONF_IHOLDDELAY = "iholddelay"
CONF_TPOWERDOWN = "tpowerdown"
CONF_ENABLE_SPREADCYCLE = "enable_spreadcycle"
CONF_TCOOL_THRESHOLD = "tcool_threshold"
CONF_TPWM_THRESHOLD = "tpwm_threshold"
CONF_STANDSTILL_MODE = "standstill_mode"


CONF_SEIMIN = "seimin"
CONF_SEDN = "sedn"
CONF_SEMAX = "semax"
CONF_SEUP = "seup"
CONF_SEMIN = "semin"
CONF_TBL = "tbl"
CONF_HEND = "hend"
CONF_HSTRT = "hstrt"

CONF_PWM_LIM = "lim"
CONF_PWM_REG = "reg"
CONF_PWM_AUTOGRAD = "autograd"
CONF_PWM_AUTOSCALE = "autoscale"
CONF_PWM_FREQ = "freq"
CONF_PWM_GRAD = "grad"
CONF_PWM_OFS = "ofs"


tmc2209_ns = cg.esphome_ns.namespace("tmc2209")
TMC2209API = tmc2209_ns.class_("TMC2209API", uart.UARTDevice)
TMC2209Component = tmc2209_ns.class_("TMC2209Component", TMC2209API, cg.Component)

DriverStatusEvent = tmc2209_ns.enum("DriverStatusEvent")

OnDriverStatusTrigger = tmc2209_ns.class_("OnDriverStatusTrigger", automation.Trigger)
OnStallTrigger = tmc2209_ns.class_("OnStallTrigger", automation.Trigger)

ConfigureAction = tmc2209_ns.class_("ConfigureAction", automation.Action)
ActivationAction = tmc2209_ns.class_("ActivationAction", automation.Action)
CurrentsAction = tmc2209_ns.class_("CurrentsAction", automation.Action)
StallGuardAction = tmc2209_ns.class_("StallGuardAction", automation.Action)
CoolConfAction = tmc2209_ns.class_("CoolConfAction", automation.Action)
ChopConfAction = tmc2209_ns.class_("ChopConfAction", automation.Action)
PWMConfAction = tmc2209_ns.class_("PWMConfAction", automation.Action)

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
            cv.Optional(CONF_VSENSE): cv.boolean,  # default OTP
            cv.Optional(CONF_OTTRIM): cv.int_range(0, 3),  # default OTP
            cv.Optional(
                CONF_RSENSE, default=0.170
            ): cv.resistance,  # default is 170 mOhm
            cv.Optional(CONF_ANALOG_SCALE, default=True): cv.boolean,
            cv.Optional(CONF_CLOCK_FREQUENCY, default=12_000_000): cv.All(
                cv.positive_int, cv.frequency
            ),
            cv.Optional(
                CONF_DRIVER_POLL_INTERVAL, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_DRIVER_STATUS): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        OnDriverStatusTrigger
                    ),
                }
            ),
            cv.Optional(CONF_ON_STALL): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnStallTrigger),
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

    if enn_pin is not None:
        cg.add_define("HAS_ENN_PIN")
        cg.add(var.set_enn_pin(await cg.gpio_pin_expression(enn_pin)))

    if diag_pin is not None:
        cg.add_define("HAS_DIAG_PIN")
        cg.add(var.set_diag_pin(await cg.gpio_pin_expression(diag_pin)))

    if index_pin is not None:
        cg.add_define("HAS_INDEX_PIN")
        cg.add(var.set_index_pin(await cg.gpio_pin_expression(index_pin)))

    if step_pin is not None:
        cg.add_define("HAS_STEP_PIN")
        cg.add(var.set_step_pin(await cg.gpio_pin_expression(step_pin)))

    if dir_pin is not None:
        cg.add_define("HAS_DIR_PIN")
        cg.add(var.set_dir_pin(await cg.gpio_pin_expression(dir_pin)))

    if (vsense := config.get(CONF_VSENSE, None)) is not None:
        cg.add_define("VSENSE", vsense)

    if (ottrim := config.get(CONF_OTTRIM, None)) is not None:
        cg.add_define("OTTRIM", ottrim)

    cg.add_define("DRIVER_POLL_INTERVAL", config[CONF_DRIVER_POLL_INTERVAL])

    for conf in config.get(CONF_ON_DRIVER_STATUS, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(DriverStatusEvent, "code")], conf)
        cg.add_define("ENABLE_DRIVER_STATUS_POLLING")

    for conf in config.get(CONF_ON_STALL, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
        cg.add_define("ENABLE_STALL_DETECTION")

    cg.add_build_flag("-std=c++17")
    cg.add_build_flag("-std=gnu++17")

    return var


def validate_tmc2209_base(config):
    return config


@automation.register_action(
    "tmc2209.configure",
    ConfigureAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_DIRECTION): cv.one_of(
                OPT_CLOCKWISE, OPT_CW, OPT_COUNTERCLOCKWISE, OPT_CCW
            ),
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 1)
            ),
            cv.Optional(CONF_INTERPOLATION): cv.templatable(cv.boolean),
            cv.Optional(CONF_ENABLE_SPREADCYCLE): cv.boolean,
            cv.Optional(CONF_TCOOL_THRESHOLD): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
            cv.Optional(CONF_TPWM_THRESHOLD): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
        }
    ),
)
def tmc2209_configure_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (dir := config.get(CONF_DIRECTION, None)) is not None:
        template_ = yield cg.templatable(
            (dir in [OPT_COUNTERCLOCKWISE, OPT_CCW]), args, bool
        )
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

    if (tcoolthrs := config.get(CONF_TCOOL_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(tcoolthrs, args, int)
        cg.add(var.set_tcool_threshold(template_))

    if (tpwmthrs := config.get(CONF_TPWM_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(tpwmthrs, args, int)
        cg.add(var.set_tpwm_threshold(template_))

    yield var


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
    "tmc2209.stallguard",
    StallGuardAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_THRESHOLD): cv.templatable(cv.int_range(min=0, max=2**8)),
        }
    ),
)
def tmc2209_stallguard_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (sgthrs := config.get(CONF_THRESHOLD, None)) is not None:
        template_ = yield cg.templatable(sgthrs, args, int)
        cg.add(var.set_stallguard_threshold(template_))

    yield var


@automation.register_action(
    "tmc2209.coolconf",
    CoolConfAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_SEIMIN): cv.All(cv.boolean, cv.int_range(min=0, max=1)),
            cv.Optional(CONF_SEMAX): cv.int_range(min=0, max=2**4, max_included=False),
            cv.Optional(CONF_SEMIN): cv.int_range(min=0, max=2**4, max_included=False),
            cv.Optional(CONF_SEDN): cv.int_range(min=0, max=2**2, max_included=False),
            cv.Optional(CONF_SEUP): cv.int_range(min=0, max=2**2, max_included=False),
        }
    ),
)
def tmc2209_coolconf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (seimin := config.get(CONF_SEIMIN, None)) is not None:
        template_ = yield cg.templatable(seimin, args, int)
        cg.add(var.set_seimin(template_))

    if (semax := config.get(CONF_SEMAX, None)) is not None:
        template_ = yield cg.templatable(semax, args, int)
        cg.add(var.set_semax(template_))

    if (semin := config.get(CONF_SEMIN, None)) is not None:
        template_ = yield cg.templatable(semin, args, int)
        cg.add(var.set_semin(template_))

    if (sedn := config.get(CONF_SEDN, None)) is not None:
        template_ = yield cg.templatable(sedn, args, int)
        cg.add(var.set_sedn(template_))

    if (seup := config.get(CONF_SEUP, None)) is not None:
        template_ = yield cg.templatable(seup, args, int)
        cg.add(var.set_seup(template_))

    yield var


@automation.register_action(
    "tmc2209.chopconf",
    ChopConfAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_TBL): cv.int_range(min=0, max=2**2, max_included=False),
            cv.Optional(CONF_HEND): cv.int_range(min=0, max=2**4, max_included=False),
            cv.Optional(CONF_HSTRT): cv.int_range(min=0, max=2**3, max_included=False),
        }
    ),
)
def tmc2209_chopconf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (tbl := config.get(CONF_TBL, None)) is not None:
        template_ = yield cg.templatable(tbl, args, int)
        cg.add(var.set_tbl(template_))

    if (hend := config.get(CONF_HEND, None)) is not None:
        template_ = yield cg.templatable(hend, args, int)
        cg.add(var.set_hend(template_))

    if (hstrt := config.get(CONF_HSTRT, None)) is not None:
        template_ = yield cg.templatable(hstrt, args, int)
        cg.add(var.set_hstrt(template_))

    yield var


@automation.register_action(
    "tmc2209.pwmconf",
    PWMConfAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2209Component),
            cv.Optional(CONF_PWM_LIM): cv.int_range(
                min=0, max=2**4, max_included=False
            ),
            cv.Optional(CONF_PWM_REG): cv.int_range(
                min=0, max=2**4, max_included=False
            ),
            cv.Optional(CONF_PWM_AUTOGRAD): cv.boolean,
            cv.Optional(CONF_PWM_AUTOSCALE): cv.boolean,
            cv.Optional(CONF_PWM_FREQ): cv.int_range(
                min=0, max=2**2, max_included=False
            ),
            cv.Optional(CONF_PWM_GRAD): cv.int_range(
                min=0, max=2**8, max_included=False
            ),
            cv.Optional(CONF_PWM_OFS): cv.int_range(
                min=0, max=2**8, max_included=False
            ),
        }
    ),
)
def tmc2209_pwmconf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if (pwmlim := config.get(CONF_PWM_LIM, None)) is not None:
        template_ = yield cg.templatable(pwmlim, args, int)
        cg.add(var.set_pwmlim(template_))

    if (pwmreg := config.get(CONF_PWM_REG, None)) is not None:
        template_ = yield cg.templatable(pwmreg, args, int)
        cg.add(var.set_pwmreg(template_))

    if (pwmautograd := config.get(CONF_PWM_AUTOGRAD, None)) is not None:
        template_ = yield cg.templatable(pwmautograd, args, bool)
        cg.add(var.set_pwmautograd(template_))

    if (pwmautoscale := config.get(CONF_PWM_AUTOSCALE, None)) is not None:
        template_ = yield cg.templatable(pwmautoscale, args, bool)
        cg.add(var.set_pwmautoscale(template_))

    if (pwmfreq := config.get(CONF_PWM_FREQ, None)) is not None:
        template_ = yield cg.templatable(pwmfreq, args, int)
        cg.add(var.set_pwmfreq(template_))

    if (pwmgrad := config.get(CONF_PWM_GRAD, None)) is not None:
        template_ = yield cg.templatable(pwmgrad, args, int)
        cg.add(var.set_pwmgrad(template_))

    if (pwmofs := config.get(CONF_PWM_OFS, None)) is not None:
        template_ = yield cg.templatable(pwmofs, args, int)
        cg.add(var.set_pwmofs(template_))

    yield var


TMC2209_FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    CONF_TMC2209, require_rx=True, require_tx=True
)
