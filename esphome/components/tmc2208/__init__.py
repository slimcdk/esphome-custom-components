import logging

from esphome.const import (
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_ID,
    CONF_STEP_PIN,
    CONF_DIR_PIN,
    CONF_DIRECTION,
    CONF_TO,
)
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import tmc2208_hub
from esphome import automation, pins
from esphome.automation import maybe_simple_id

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc2208_hub"]

CONF_TMC2208 = "tmc2208"
CONF_TMC2208_ID = "tmc2208_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"
CONF_CLOCK_FREQUENCY = "clock_frequency"
CONF_OTTRIM = "ottrim"
CONF_VSENSE = "vsense"  # true lowers power dissipation in sense resistors
CONF_RSENSE = "rsense"  # sense resistors
CONF_ANALOG_CURRENT_SCALE = "analog_current_scale"
CONF_INCLUDE_REGISTERS = "config_dump_include_registers"
CONF_ON_DRIVER_STATUS = "on_status"


# used in actions
CONF_MICROSTEPS = "microsteps"  # CHOPCONF.mres
CONF_INTERPOLATION = "interpolation"  # CHOPCONF.intpol
CONF_IRUN = "irun"
CONF_RUN_CURRENT = "run_current"  # translates to IRUN
CONF_IHOLD = "ihold"
CONF_HOLD_CURRENT = "hold_current"  # translates to IHOLD
CONF_IHOLDDELAY = "iholddelay"
CONF_TPOWERDOWN = "tpowerdown"
CONF_ENABLE_SPREADCYCLE = "enable_spreadcycle"
CONF_TPWM_THRESHOLD = "tpwm_threshold"
CONF_STANDSTILL_MODE = "standstill_mode"
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
CONF_RESTORE_TOFF = "restore_toff"


tmc2208_ns = cg.esphome_ns.namespace("tmc2208")
TMC2208API = tmc2208_ns.class_(
    "TMC2208API", cg.Parented.template(tmc2208_hub.TMC2208Hub)
)
TMC2208Component = tmc2208_ns.class_("TMC2208Component", TMC2208API, cg.Component)

DriverStatusEvent = tmc2208_ns.enum("DriverStatusEvent")
StandstillMode = tmc2208_ns.enum("StandstillMode")
ShaftDirection = tmc2208_ns.enum("ShaftDirection")

OnDriverStatusTrigger = tmc2208_ns.class_("OnDriverStatusTrigger", automation.Trigger)
OnStallTrigger = tmc2208_ns.class_("OnStallTrigger", automation.Trigger)

ConfigureAction = tmc2208_ns.class_("ConfigureAction", automation.Action)
ActivationAction = tmc2208_ns.class_("ActivationAction", automation.Action)
CurrentsAction = tmc2208_ns.class_("CurrentsAction", automation.Action)
StallGuardAction = tmc2208_ns.class_("StallGuardAction", automation.Action)
CoolConfAction = tmc2208_ns.class_("CoolConfAction", automation.Action)
ChopConfAction = tmc2208_ns.class_("ChopConfAction", automation.Action)
PWMConfAction = tmc2208_ns.class_("PWMConfAction", automation.Action)
SyncAction = tmc2208_ns.class_("SyncAction", automation.Action)


STANDSTILL_MODES = {
    "normal": StandstillMode.NORMAL,
    "freewheeling": StandstillMode.FREEWHEELING,
    "coil_short_ls": StandstillMode.COIL_SHORT_LS,
    "coil_short_hs": StandstillMode.COIL_SHORT_HS,
}

SHAFT_DIRECTIONS = {
    "clockwise": ShaftDirection.CLOCKWISE,
    "cw": ShaftDirection.CLOCKWISE,
    "counterclockwise": ShaftDirection.COUNTERCLOCKWISE,
    "ccw": ShaftDirection.COUNTERCLOCKWISE,
}


DEVICE_SCHEMA = cv.Schema({cv.GenerateID(CONF_TMC2208_ID): cv.use_id(TMC2208Component)})

TMC2208_BASE_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ENN_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_STEP_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_DIR_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
        cv.Optional(CONF_VSENSE): cv.boolean,  # default OTP
        cv.Optional(CONF_OTTRIM): cv.int_range(0, 3),  # default OTP
        cv.Optional(CONF_RSENSE): cv.resistance,  # default is rdson
        cv.Optional(CONF_ANALOG_CURRENT_SCALE, default=False): cv.boolean,
        cv.Optional(CONF_CLOCK_FREQUENCY, default=12_000_000): cv.All(
            cv.positive_int, cv.frequency
        ),
        cv.Optional(CONF_ON_DRIVER_STATUS): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnDriverStatusTrigger),
            }
        ),
        cv.Optional(CONF_INCLUDE_REGISTERS, default=False): cv.boolean,
    },
).extend(cv.COMPONENT_SCHEMA, tmc2208_hub.TMC2208_HUB_DEVICE_SCHEMA)


async def register_tmc2208_base(var, config):

    await cg.register_component(var, config)
    await tmc2208_hub.register_tmc2208_hub_device(var, config)

    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_clk_freq(config[CONF_CLOCK_FREQUENCY]))
    cg.add(var.set_analog_current_scale(config[CONF_ANALOG_CURRENT_SCALE]))
    cg.add(var.set_config_dump_include_registers(config[CONF_INCLUDE_REGISTERS]))

    if (enn_pin := config.get(CONF_ENN_PIN, None)) is not None:
        cg.add(var.set_enn_pin(await cg.gpio_pin_expression(enn_pin)))

    if (diag_pin := config.get(CONF_DIAG_PIN, None)) is not None:
        cg.add(var.set_diag_pin(await cg.gpio_pin_expression(diag_pin)))

    if (index_pin := config.get(CONF_INDEX_PIN, None)) is not None:
        cg.add(var.set_index_pin(await cg.gpio_pin_expression(index_pin)))

    if (step_pin := config.get(CONF_STEP_PIN, None)) is not None:
        cg.add(var.set_step_pin(await cg.gpio_pin_expression(step_pin)))

    if (dir_pin := config.get(CONF_DIR_PIN, None)) is not None:
        cg.add(var.set_dir_pin(await cg.gpio_pin_expression(dir_pin)))

    if (rsense := config.get(CONF_RSENSE, None)) is not None:
        cg.add(var.set_rsense(rsense))

    if (vsense := config.get(CONF_VSENSE, None)) is not None:
        cg.add(var.set_vsense(vsense))
        if CONF_RSENSE not in config and vsense is False:
            _LOGGER.warning(
                "High heat dissipation (`vsense: False`) when using RDSon / internal current sensing",
            )

    if (ottrim := config.get(CONF_OTTRIM, None)) is not None:
        cg.add(var.set_ottrim(ottrim))

    for conf in config.get(CONF_ON_DRIVER_STATUS, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(DriverStatusEvent, "code")], conf)
        cg.add(var.set_enable_driver_health_check(True))

    cg.add_build_flag("-std=c++17")
    cg.add_build_flag("-std=gnu++17")

    return var


def validate_tmc2208_base(config):
    return config


@automation.register_action(
    "tmc2208.enable",
    ActivationAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Optional(CONF_RESTORE_TOFF, default=True): cv.boolean,
        }
    ),
)
async def tmc2208_enable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(True))
    cg.add(var.set_toff_recovery(config[CONF_RESTORE_TOFF]))
    return var


@automation.register_action(
    "tmc2208.disable",
    ActivationAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Optional(CONF_RESTORE_TOFF, default=True): cv.boolean,
        }
    ),
)
async def tmc2208_disable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(False))
    cg.add(var.set_toff_recovery(config[CONF_RESTORE_TOFF]))
    return var


@automation.register_action(
    "tmc2208.configure",
    ConfigureAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Optional(CONF_DIRECTION): cv.templatable(cv.enum(SHAFT_DIRECTIONS)),
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 1, int=True)
            ),
            cv.Optional(CONF_INTERPOLATION): cv.templatable(cv.boolean),
            cv.Optional(CONF_ENABLE_SPREADCYCLE): cv.templatable(cv.boolean),
            cv.Optional(CONF_TPWM_THRESHOLD): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
        }
    ),
)
async def tmc2208_configure_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if (dir := config.get(CONF_DIRECTION, None)) is not None:
        template_ = await cg.templatable(dir, args, ShaftDirection)
        cg.add(var.set_inverse_direction(template_))

    if (microsteps := config.get(CONF_MICROSTEPS, None)) is not None:
        template_ = await cg.templatable(microsteps, args, cg.int16)
        cg.add(var.set_microsteps(template_))

    if (interpolation := config.get(CONF_INTERPOLATION, None)) is not None:
        template_ = await cg.templatable(interpolation, args, cg.bool_)
        cg.add(var.set_microstep_interpolation(template_))

    if (en_spreadcycle := config.get(CONF_ENABLE_SPREADCYCLE, None)) is not None:
        template_ = await cg.templatable(en_spreadcycle, args, cg.bool_)
        cg.add(var.set_enable_spreadcycle(template_))

    if (tpwmthrs := config.get(CONF_TPWM_THRESHOLD, None)) is not None:
        template_ = await cg.templatable(tpwmthrs, args, cg.uint32)
        cg.add(var.set_tpwm_threshold(template_))

    return var


@automation.register_action(
    "tmc2208.currents",
    CurrentsAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Exclusive(CONF_RUN_CURRENT, "run current"): cv.templatable(
                cv.All(cv.current, cv.positive_not_null_float)
            ),
            cv.Exclusive(CONF_IRUN, "run current"): cv.templatable(
                cv.int_range(min=0, max=31)
            ),
            cv.Exclusive(CONF_HOLD_CURRENT, "hold current"): cv.templatable(
                cv.All(cv.current, cv.positive_float)
            ),
            cv.Exclusive(CONF_IHOLD, "hold current"): cv.templatable(
                cv.int_range(min=0, max=31)
            ),
            cv.Optional(CONF_STANDSTILL_MODE): cv.templatable(
                cv.enum(STANDSTILL_MODES, string=True)
            ),
            cv.Optional(CONF_IHOLDDELAY): cv.templatable(cv.int_range(0, 15)),
            cv.Optional(CONF_TPOWERDOWN): cv.templatable(cv.int_range(0, 255)),
        }
    ),
)
async def tmc2208_currents_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if (standstill_mode := config.get(CONF_STANDSTILL_MODE, None)) is not None:
        template_ = await cg.templatable(standstill_mode, args, StandstillMode)
        cg.add(var.set_standstill_mode(template_))

    if (run_current := config.get(CONF_RUN_CURRENT, None)) is not None:
        template_ = await cg.templatable(run_current, args, cg.float_)
        cg.add(var.set_run_current(template_))

    if (irun := config.get(CONF_IRUN, None)) is not None:
        template_ = await cg.templatable(irun, args, cg.uint8)
        cg.add(var.set_irun(template_))

    if (hold_current := config.get(CONF_HOLD_CURRENT, None)) is not None:
        template_ = await cg.templatable(hold_current, args, cg.float_)
        cg.add(var.set_hold_current(template_))

    if (ihold := config.get(CONF_IHOLD, None)) is not None:
        template_ = await cg.templatable(ihold, args, cg.uint8)
        cg.add(var.set_ihold(template_))

    if (iholddelay := config.get(CONF_IHOLDDELAY, None)) is not None:
        template_ = await cg.templatable(iholddelay, args, cg.uint8)
        cg.add(var.set_iholddelay(template_))

    if (tpowerdown := config.get(CONF_TPOWERDOWN, None)) is not None:
        template_ = await cg.templatable(tpowerdown, args, cg.uint8)
        cg.add(var.set_tpowerdown(template_))

    return var


@automation.register_action(
    "tmc2208.chopconf",
    ChopConfAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Optional(CONF_TBL): cv.templatable(
                cv.int_range(min=0, max=2**2, max_included=False)
            ),
            cv.Optional(CONF_HEND): cv.templatable(
                cv.int_range(min=0, max=2**4, max_included=False)
            ),
            cv.Optional(CONF_HSTRT): cv.templatable(
                cv.int_range(min=0, max=2**3, max_included=False)
            ),
        }
    ),
)
async def tmc2208_chopconf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if (tbl := config.get(CONF_TBL, None)) is not None:
        template_ = await cg.templatable(tbl, args, cg.uint8)
        cg.add(var.set_tbl(template_))

    if (hend := config.get(CONF_HEND, None)) is not None:
        template_ = await cg.templatable(hend, args, cg.uint8)
        cg.add(var.set_hend(template_))

    if (hstrt := config.get(CONF_HSTRT, None)) is not None:
        template_ = await cg.templatable(hstrt, args, cg.uint8)
        cg.add(var.set_hstrt(template_))

    return var


@automation.register_action(
    "tmc2208.pwmconf",
    PWMConfAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Optional(CONF_PWM_LIM): cv.templatable(
                cv.int_range(min=0, max=2**4, max_included=False)
            ),
            cv.Optional(CONF_PWM_REG): cv.templatable(
                cv.int_range(min=0, max=2**4, max_included=False)
            ),
            cv.Optional(CONF_PWM_FREQ): cv.templatable(
                cv.int_range(min=0, max=2**2, max_included=False)
            ),
            cv.Optional(CONF_PWM_GRAD): cv.templatable(
                cv.int_range(min=0, max=2**8, max_included=False)
            ),
            cv.Optional(CONF_PWM_OFS): cv.templatable(
                cv.int_range(min=0, max=2**8, max_included=False)
            ),
            cv.Optional(CONF_PWM_AUTOGRAD): cv.templatable(cv.boolean),
            cv.Optional(CONF_PWM_AUTOSCALE): cv.templatable(cv.boolean),
        }
    ),
)
async def tmc2208_pwmconf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if (pwmlim := config.get(CONF_PWM_LIM, None)) is not None:
        template_ = await cg.templatable(pwmlim, args, cg.uint8)
        cg.add(var.set_pwmlim(template_))

    if (pwmreg := config.get(CONF_PWM_REG, None)) is not None:
        template_ = await cg.templatable(pwmreg, args, cg.uint8)
        cg.add(var.set_pwmreg(template_))

    if (pwmfreq := config.get(CONF_PWM_FREQ, None)) is not None:
        template_ = await cg.templatable(pwmfreq, args, cg.uint8)
        cg.add(var.set_pwmfreq(template_))

    if (pwmgrad := config.get(CONF_PWM_GRAD, None)) is not None:
        template_ = await cg.templatable(pwmgrad, args, cg.uint8)
        cg.add(var.set_pwmgrad(template_))

    if (pwmofs := config.get(CONF_PWM_OFS, None)) is not None:
        template_ = await cg.templatable(pwmofs, args, cg.uint8)
        cg.add(var.set_pwmofs(template_))

    if (pwmautograd := config.get(CONF_PWM_AUTOGRAD, None)) is not None:
        template_ = await cg.templatable(pwmautograd, args, cg.bool_)
        cg.add(var.set_pwmautograd(template_))

    if (pwmautoscale := config.get(CONF_PWM_AUTOSCALE, None)) is not None:
        template_ = await cg.templatable(pwmautoscale, args, cg.bool_)
        cg.add(var.set_pwmautoscale(template_))

    return var


@automation.register_action(
    "tmc2208.sync",
    SyncAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TMC2208Component),
            cv.Required(CONF_TO): cv.All(
                cv.ensure_list(cv.use_id(TMC2208Component)), cv.Length(min=1)
            ),
        }
    ),
)
async def tmc2208_sync_to_code(config, action_id, template_arg, args):

    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if config[CONF_ID] in config[CONF_TO]:
        _LOGGER.error("tmc2208.sync for %s is syncing to self", config[CONF_ID])

    if len(config[CONF_TO]) != len(set(config[CONF_TO])):
        _LOGGER.warning("tmc2208.sync for %s has duplicate references", config[CONF_ID])

    template_ = await cg.templatable(
        [await cg.get_variable(id) for id in config[CONF_TO]],
        args,
        cg.std_vector.template(TMC2208Component),
    )
    cg.add(var.set_drivers(template_))

    return var
