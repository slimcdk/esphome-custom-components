import logging

from esphome.const import (
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_ID,
    CONF_STEP_PIN,
    CONF_DIR_PIN,
    CONF_DIRECTION,
    CONF_THRESHOLD,
    CONF_TO,
    CONF_VARIANT,
)
import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import tmc22xx_hub
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.core.entity_helpers import inherit_property_from
import esphome.final_validate as fv


_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc22xx_hub"]

CONF_TMC22XX = "tmc22xx"
CONF_TMC22XX_ID = "tmc22xx_id"

VARIANT_TMC2202 = "tmc2202"
VARIANT_TMC2208 = "tmc2208"
VARIANT_TMC2209 = "tmc2209"
VARIANT_TMC2224 = "tmc2224"
VARIANT_TMC2225 = "tmc2225"
VARIANT_TMC2226 = "tmc2226"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"
CONF_SELECT_PIN = "select_pin"
CONF_CLOCK_FREQUENCY = "clock_frequency"
CONF_OTTRIM = "ottrim"
CONF_VSENSE = "vsense"
CONF_RSENSE = "rsense"
CONF_ANALOG_CURRENT_SCALE = "analog_current_scale"
CONF_ON_STALL = "on_stall"
CONF_ON_DRIVER_STATUS = "on_status"

# used in actions
CONF_MICROSTEPS = "microsteps"
CONF_INTERPOLATION = "interpolation"
CONF_IRUN = "irun"
CONF_RUN_CURRENT = "run_current"
CONF_IHOLD = "ihold"
CONF_HOLD_CURRENT = "hold_current"
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
CONF_RESTORE_TOFF = "restore_toff"


tmc22xx_ns = cg.esphome_ns.namespace("tmc22xx")
TMC22XXAPI = tmc22xx_ns.class_(
    "TMC22XXAPI", cg.Parented.template(tmc22xx_hub.TMC22XXHub)
)
TMC22XXComponent = tmc22xx_ns.class_("TMC22XXComponent", TMC22XXAPI, cg.Component)
# TMC2202Component = tmc22xx_ns.class_("TMC2202Component", TMC22XXComponent)
# TMC2208Component = tmc22xx_ns.class_("TMC2208Component", TMC22XXComponent)
# TMC2209Component = tmc22xx_ns.class_("TMC2209Component", TMC22XXComponent)
# TMC2226Component = tmc22xx_ns.class_("TMC2226Component", TMC22XXComponent)
# TMC2224Component = tmc22xx_ns.class_("TMC2224Component", TMC22XXComponent)
# TMC2225Component = tmc22xx_ns.class_("TMC2225Component", TMC22XXComponent)

DriverStatusEvent = tmc22xx_ns.enum("DriverStatusEvent")

OnDriverStatusTrigger = tmc22xx_ns.class_("OnDriverStatusTrigger", automation.Trigger)
OnStallTrigger = tmc22xx_ns.class_("OnStallTrigger", automation.Trigger)


action_base = (automation.Action, cg.Parented.template(TMC22XXComponent))
ConfigureAction = tmc22xx_ns.class_("ConfigureAction", *action_base)
ActivationAction = tmc22xx_ns.class_("ActivationAction", *action_base)
CurrentsAction = tmc22xx_ns.class_("CurrentsAction", *action_base)
StallGuardAction = tmc22xx_ns.class_("StallGuardAction", *action_base)
CoolConfAction = tmc22xx_ns.class_("CoolConfAction", *action_base)
ChopConfAction = tmc22xx_ns.class_("ChopConfAction", *action_base)
PWMConfAction = tmc22xx_ns.class_("PWMConfAction", *action_base)
SyncAction = tmc22xx_ns.class_("SyncAction", *action_base)

DriverVariant = tmc22xx_ns.enum("Variant")
DRIVER_VARIANTS = {
    VARIANT_TMC2202: DriverVariant.TMC2202,
    VARIANT_TMC2208: DriverVariant.TMC2208,
    VARIANT_TMC2209: DriverVariant.TMC2209,
    VARIANT_TMC2224: DriverVariant.TMC2224,
    VARIANT_TMC2225: DriverVariant.TMC2225,
    VARIANT_TMC2226: DriverVariant.TMC2226,
}

StandstillMode = tmc22xx_ns.enum("StandstillMode")
STANDSTILL_MODES = {
    "normal": StandstillMode.NORMAL,
    "freewheeling": StandstillMode.FREEWHEELING,
    "coil_short_ls": StandstillMode.COIL_SHORT_LS,
    "coil_short_hs": StandstillMode.COIL_SHORT_HS,
}

ShaftDirection = tmc22xx_ns.enum("ShaftDirection")
SHAFT_DIRECTIONS = {
    "clockwise": ShaftDirection.CLOCKWISE,
    "cw": ShaftDirection.CLOCKWISE,
    "counterclockwise": ShaftDirection.COUNTERCLOCKWISE,
    "ccw": ShaftDirection.COUNTERCLOCKWISE,
}


def _on_stall_invalid(variant):
    def handler(value):
        raise cv.Invalid(
            f"[{CONF_ON_STALL}]: {variant} does not support stall detection"
        )

    return handler


DEVICE_SCHEMA = cv.Schema({cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XXComponent)})

TMC22XX_BASE_CONFIG_SCHEMA = tmc22xx_hub.TMC22XX_HUB_DEVICE_SCHEMA.extend(
    cv.COMPONENT_SCHEMA,
    cv.Schema(
        {
            # api
            cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
            # io
            cv.Optional(CONF_ENN_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_STEP_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SELECT_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CLOCK_FREQUENCY, default=12_000_000): cv.All(
                cv.positive_int, cv.frequency
            ),
            # settings
            cv.Optional(CONF_ANALOG_CURRENT_SCALE, default=False): cv.boolean,
            cv.Optional(CONF_VSENSE): cv.boolean,  # default OTP
            cv.Optional(CONF_OTTRIM): cv.int_range(0, 3),  # default OTP
            cv.Optional(CONF_RSENSE): cv.resistance,  # default is rdson
            # cv.Optional(CONF_INCLUDE_REGISTERS, default=False): cv.boolean,
            cv.Optional(CONF_ON_DRIVER_STATUS): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnDriverStatusTrigger)}
            ),
        },
    ),
)

# tmc2209 has stall detection
TMC22XX_STALL_TRIGGER_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ON_STALL): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnStallTrigger)}
        ),
    },
)


def _build_typed_schema(tmc2202, tmc2208, tmc2209, tmc2224, tmc2225, tmc2226, **kwargs):

    extend = kwargs.pop("extend", cv.Schema({}))

    return cv.typed_schema(
        {
            VARIANT_TMC2202: TMC22XX_BASE_CONFIG_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(tmc2202),
                        cv.Optional(CONF_ON_STALL): _on_stall_invalid(VARIANT_TMC2202),
                    }
                ),
                extend,
            ),
            VARIANT_TMC2208: TMC22XX_BASE_CONFIG_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(tmc2208),
                        cv.Optional(CONF_ON_STALL): _on_stall_invalid(VARIANT_TMC2208),
                    }
                ),
                extend,
            ),
            VARIANT_TMC2209: TMC22XX_BASE_CONFIG_SCHEMA.extend(
                cv.Schema({cv.GenerateID(): cv.declare_id(tmc2209)}),
                TMC22XX_STALL_TRIGGER_CONFIG_SCHEMA,
                extend,
            ),
            VARIANT_TMC2224: TMC22XX_BASE_CONFIG_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(tmc2224),
                        cv.Optional(CONF_ON_STALL): _on_stall_invalid(VARIANT_TMC2224),
                    }
                ),
                extend,
            ),
            VARIANT_TMC2225: TMC22XX_BASE_CONFIG_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(tmc2225),
                        cv.Optional(CONF_ON_STALL): _on_stall_invalid(VARIANT_TMC2225),
                    }
                ),
                extend,
            ),
            VARIANT_TMC2226: TMC22XX_BASE_CONFIG_SCHEMA.extend(
                cv.Schema({cv.GenerateID(): cv.declare_id(tmc2226)}),
                TMC22XX_STALL_TRIGGER_CONFIG_SCHEMA,
                extend,
            ),
        },
        key=CONF_VARIANT,
    )


TMC22XX_CONFIG_SCHEMA = _build_typed_schema(
    tmc2202=TMC22XXComponent,  # TMC2202Component,
    tmc2208=TMC22XXComponent,  # TMC2208Component,
    tmc2209=TMC22XXComponent,  # TMC2209Component,
    tmc2224=TMC22XXComponent,  # TMC2224Component,
    tmc2225=TMC22XXComponent,  # TMC2225Component,
    tmc2226=TMC22XXComponent,  # TMC2226Component,
)


async def register_tmc22xx_base(var, config):

    cg.add_build_flag("-std=c++17")
    cg.add_build_flag("-std=gnu++17")

    await cg.register_component(var, config)
    hub = await tmc22xx_hub.register_tmc22xx_hub_device(var, config)
    cg.add(hub.notify_device_in_hub_(str(config[CONF_ID]), config[CONF_ADDRESS]))

    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_clk_freq(config[CONF_CLOCK_FREQUENCY]))
    cg.add(var.set_analog_current_scale(config[CONF_ANALOG_CURRENT_SCALE]))

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
    if (sel_pin := config.get(CONF_SELECT_PIN, None)) is not None:
        cg.add(var.set_sel_pin(await cg.gpio_pin_expression(sel_pin)))

    if (ottrim := config.get(CONF_OTTRIM, None)) is not None:
        cg.add(var.set_ottrim(ottrim))
    if (rsense := config.get(CONF_RSENSE, None)) is not None:
        cg.add(var.set_rsense(rsense))
    if (vsense := config.get(CONF_VSENSE, None)) is not None:
        cg.add(var.set_vsense(vsense))
        if CONF_RSENSE not in config and vsense is False:
            _LOGGER.warning(
                "High heat dissipation (`vsense: False`) when using RDSon / internal current sensing",
            )

    for conf in config.get(CONF_ON_DRIVER_STATUS, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(DriverStatusEvent, "code")], conf)
        cg.add(var.set_enable_driver_health_check(True))

    for conf in config.get(CONF_ON_STALL, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
        cg.add(var.set_enable_stall_detection(True))

    return var


ACTION_ACTIVATE_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
        cv.Optional(CONF_RESTORE_TOFF, default=True): cv.boolean,
    }
)


@automation.register_action("tmc2202.enable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2208.enable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2209.enable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2224.enable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2225.enable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2226.enable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
async def tmc22xx_enable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(True))
    cg.add(var.set_toff_recovery(config[CONF_RESTORE_TOFF]))
    return var


@automation.register_action("tmc2202.disable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2208.disable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2209.disable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2224.disable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2225.disable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
@automation.register_action("tmc2226.disable", ActivationAction, ACTION_ACTIVATE_SCHEMA)
async def tmc22xx_disable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_activate(False))
    cg.add(var.set_toff_recovery(config[CONF_RESTORE_TOFF]))
    return var


ACTION_CONFIG_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
        cv.Optional(CONF_DIRECTION): cv.templatable(cv.enum(SHAFT_DIRECTIONS)),
        cv.Optional(CONF_MICROSTEPS): cv.templatable(
            cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 1, int=True)
        ),
        cv.Optional(CONF_INTERPOLATION): cv.templatable(cv.boolean),
        cv.Optional(CONF_ENABLE_SPREADCYCLE): cv.templatable(cv.boolean),
        cv.Optional(CONF_TCOOL_THRESHOLD): cv.templatable(
            cv.int_range(min=0, max=2**20, max_included=False)
        ),
        cv.Optional(CONF_TPWM_THRESHOLD): cv.templatable(
            cv.int_range(min=0, max=2**20, max_included=False)
        ),
    }
)


@automation.register_action("tmc2202.configure", ConfigureAction, ACTION_CONFIG_SCHEMA)
@automation.register_action("tmc2208.configure", ConfigureAction, ACTION_CONFIG_SCHEMA)
@automation.register_action("tmc2209.configure", ConfigureAction, ACTION_CONFIG_SCHEMA)
@automation.register_action("tmc2224.configure", ConfigureAction, ACTION_CONFIG_SCHEMA)
@automation.register_action("tmc2225.configure", ConfigureAction, ACTION_CONFIG_SCHEMA)
@automation.register_action("tmc2226.configure", ConfigureAction, ACTION_CONFIG_SCHEMA)
async def tmc22xx_configure_to_code(config, action_id, template_arg, args):
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

    if (tcoolthrs := config.get(CONF_TCOOL_THRESHOLD, None)) is not None:
        template_ = await cg.templatable(tcoolthrs, args, cg.uint32)
        cg.add(var.set_tcool_threshold(template_))

    if (tpwmthrs := config.get(CONF_TPWM_THRESHOLD, None)) is not None:
        template_ = await cg.templatable(tpwmthrs, args, cg.uint32)
        cg.add(var.set_tpwm_threshold(template_))

    return var


ACTION_CURRENT_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
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
)


@automation.register_action("tmc2202.currents", CurrentsAction, ACTION_CURRENT_SCHEMA)
@automation.register_action("tmc2208.currents", CurrentsAction, ACTION_CURRENT_SCHEMA)
@automation.register_action("tmc2209.currents", CurrentsAction, ACTION_CURRENT_SCHEMA)
@automation.register_action("tmc2224.currents", CurrentsAction, ACTION_CURRENT_SCHEMA)
@automation.register_action("tmc2225.currents", CurrentsAction, ACTION_CURRENT_SCHEMA)
@automation.register_action("tmc2226.currents", CurrentsAction, ACTION_CURRENT_SCHEMA)
async def tmc22xx_currents_to_code(config, action_id, template_arg, args):
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


ACTION_SG_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
        cv.Optional(CONF_THRESHOLD): cv.templatable(
            cv.int_range(min=0, max=2**8, max_included=False)
        ),
    },
)


@automation.register_action("tmc2209.stallguard", StallGuardAction, ACTION_SG_SCHEMA)
@automation.register_action("tmc2226.stallguard", StallGuardAction, ACTION_SG_SCHEMA)
async def tmc22xx_stallguard_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if (sgthrs := config.get(CONF_THRESHOLD, None)) is not None:
        template_ = await cg.templatable(sgthrs, args, cg.uint8)
        cg.add(var.set_stallguard_threshold(template_))

    return var


ACTION_COOLCONF_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
        cv.Optional(CONF_SEIMIN): cv.templatable(
            cv.All(cv.boolean, cv.int_range(min=0, max=1))
        ),
        cv.Optional(CONF_SEMAX): cv.templatable(
            cv.int_range(min=0, max=2**4, max_included=False)
        ),
        cv.Optional(CONF_SEMIN): cv.templatable(
            cv.int_range(min=0, max=2**4, max_included=False)
        ),
        cv.Optional(CONF_SEDN): cv.templatable(
            cv.int_range(min=0, max=2**2, max_included=False)
        ),
        cv.Optional(CONF_SEUP): cv.templatable(
            cv.int_range(min=0, max=2**2, max_included=False)
        ),
    }
)


@automation.register_action("tmc2202.coolconf", CoolConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2208.coolconf", CoolConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2209.coolconf", CoolConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2224.coolconf", CoolConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2225.coolconf", CoolConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2226.coolconf", CoolConfAction, ACTION_COOLCONF_SCHEMA)
async def tmc22xx_coolconf_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if (seimin := config.get(CONF_SEIMIN, None)) is not None:
        template_ = await cg.templatable(seimin, args, cg.uint8)
        cg.add(var.set_seimin(template_))

    if (semax := config.get(CONF_SEMAX, None)) is not None:
        template_ = await cg.templatable(semax, args, cg.uint8)
        cg.add(var.set_semax(template_))

    if (semin := config.get(CONF_SEMIN, None)) is not None:
        template_ = await cg.templatable(semin, args, cg.uint8)
        cg.add(var.set_semin(template_))

    if (sedn := config.get(CONF_SEDN, None)) is not None:
        template_ = await cg.templatable(sedn, args, cg.uint8)
        cg.add(var.set_sedn(template_))

    if (seup := config.get(CONF_SEUP, None)) is not None:
        template_ = await cg.templatable(seup, args, cg.uint8)
        cg.add(var.set_seup(template_))

    return var


ACTION_COOLCONF_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
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
)


@automation.register_action("tmc2202.chopconf", ChopConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2208.chopconf", ChopConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2209.chopconf", ChopConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2224.chopconf", ChopConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2225.chopconf", ChopConfAction, ACTION_COOLCONF_SCHEMA)
@automation.register_action("tmc2226.chopconf", ChopConfAction, ACTION_COOLCONF_SCHEMA)
async def tmc22xx_chopconf_to_code(config, action_id, template_arg, args):
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


ACTION_PWMCONF_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
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
)


@automation.register_action("tmc2202.pwmconf", PWMConfAction, ACTION_PWMCONF_SCHEMA)
@automation.register_action("tmc2208.pwmconf", PWMConfAction, ACTION_PWMCONF_SCHEMA)
@automation.register_action("tmc2209.pwmconf", PWMConfAction, ACTION_PWMCONF_SCHEMA)
@automation.register_action("tmc2224.pwmconf", PWMConfAction, ACTION_PWMCONF_SCHEMA)
@automation.register_action("tmc2225.pwmconf", PWMConfAction, ACTION_PWMCONF_SCHEMA)
@automation.register_action("tmc2226.pwmconf", PWMConfAction, ACTION_PWMCONF_SCHEMA)
async def tmc22xx_pwmconf_to_code(config, action_id, template_arg, args):
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


ACTION_SYNC_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(TMC22XXComponent),
        cv.Required(CONF_TO): cv.All(
            cv.ensure_list(cv.use_id(TMC22XXComponent)), cv.Length(min=1)
        ),
    }
)


@automation.register_action("tmc2202.sync", SyncAction, ACTION_SYNC_SCHEMA)
@automation.register_action("tmc2208.sync", SyncAction, ACTION_SYNC_SCHEMA)
@automation.register_action("tmc2209.sync", SyncAction, ACTION_SYNC_SCHEMA)
@automation.register_action("tmc2224.sync", SyncAction, ACTION_SYNC_SCHEMA)
@automation.register_action("tmc2225.sync", SyncAction, ACTION_SYNC_SCHEMA)
@automation.register_action("tmc2226.sync", SyncAction, ACTION_SYNC_SCHEMA)
async def tmc22xx_sync_to_code(config, action_id, template_arg, args):

    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    if config[CONF_ID] in config[CONF_TO]:
        _LOGGER.error("tmc22xx.sync for %s is syncing to self", config[CONF_ID])

    if len(config[CONF_TO]) != len(set(config[CONF_TO])):
        _LOGGER.warning("tmc22xx.sync for %s has duplicate references", config[CONF_ID])

    template_ = await cg.templatable(
        [await cg.get_variable(id) for id in config[CONF_TO]],
        args,
        cg.std_vector.template(TMC22XXComponent),
    )
    cg.add(var.set_drivers(template_))

    return var
