from esphome import automation, pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.components import uart
from esphome.const import CONF_TRIGGER_ID, CONF_ADDRESS, CONF_ID


CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["uart"]
MULTI_CONF = False  # can be multiconf when tmc2209_cache() is implemented
MULTI_CONF_NO_DEFAULT = True

CONF_TMC2209 = "tmc2209"
CONF_TMC2209_ID = "tmc2209_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"

CONF_INVERSE_DIRECTION = "inverse_direction"

CONF_VELOCITY = "velocity"

CONF_VACTUAL = "vactual"
CONF_MICROSTEPS = "microsteps"  # CHOPCONF.mres
CONF_INTERPOLATION = "interpolation"  # CHOPCONF.intpol
CONF_COOLSTEP_TCOOLTHRS = "coolstep_tcoolthrs"
CONF_STALLGUARD_SGTHRS = "stallguard_sgthrs"

CONF_RMS_CURRENT = "rms_current"
CONF_RMS_CURRENT_HOLD_SCALE = "rms_current_hold_scale"

CONF_HOLD_CURRENT_DELAY = "hold_current_delay"
CONF_POWER_DOWN_DELAY = "power_down_delay"
CONF_TSTEP = "tstep"
CONF_TPWMTHRS = "tpwmthrs"
CONF_RSENSE = "rsense"
CONF_OSCILLATOR_FREQUENCY = "oscillator_freq"

# CONF_COOLCONF_SEIMIN = "coolstep_seimin"
# CONF_COOLCONF_SEDN1 = "coolstep_sedn1"
# CONF_COOLCONF_SEDN0 = "coolstep_sedn0"
# CONF_COOLCONF_SEDN1 = "coolstep_sedn1"
# CONF_COOLCONF_SEMAX3 = "coolstep_semax3"
# CONF_COOLCONF_SEMAX2 = "coolstep_semax2"
# CONF_COOLCONF_SEMAX1 = "coolstep_semax1"
# CONF_COOLCONF_SEMAX0 = "coolstep_semax0"
# CONF_COOLCONF_SEUP1 = "coolstep_seup1"
# CONF_COOLCONF_SEUP0 = "coolstep_seup0"
# CONF_COOLCONF_SEMIN3 = "coolstep_semin3"
# CONF_COOLCONF_SEMIN2 = "coolstep_semin2"
# CONF_COOLCONF_SEMIN1 = "coolstep_semin1"
# CONF_COOLCONF_SEMIN0 = "coolstep_semin0"

CONF_ON_ALERT = "on_alert"

tmc2209_ns = cg.esphome_ns.namespace("tmc2209")
TMC2209 = tmc2209_ns.class_("TMC2209", cg.Component, uart.UARTDevice)


TMC2209OnAlertTrigger = tmc2209_ns.class_("TMC2209OnAlertTrigger", automation.Trigger)
TMC2209ConfigureAction = tmc2209_ns.class_("TMC2209ConfigureAction", automation.Action)
DriverEvent = tmc2209_ns.enum("DriverEvent")

TMC2209_CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209),
    }
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC2209),
            cv.Optional(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0x00): cv.uint8_t,
            cv.Optional(CONF_RSENSE, default=0): cv.resistance,
            cv.Optional(CONF_OSCILLATOR_FREQUENCY, default=12_000_000): cv.frequency,
            cv.Optional(CONF_ON_ALERT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        TMC2209OnAlertTrigger
                    ),
                }
            ),
        },
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_ADDRESS])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_INDEX_PIN in config:
        cg.add(var.set_index_pin(await cg.gpio_pin_expression(config[CONF_INDEX_PIN])))
        cg.add_define("USE_INDEX_PIN")

    if CONF_DIAG_PIN in config:
        cg.add_define("USE_DIAG_PIN")
        cg.add(var.set_diag_pin(await cg.gpio_pin_expression(config[CONF_DIAG_PIN])))

    cg.add(var.set_oscillator_frequency(config[CONF_OSCILLATOR_FREQUENCY]))
    cg.add(var.set_rsense(config[CONF_RSENSE], CONF_RSENSE not in config))

    for conf in config.get(CONF_ON_ALERT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(DriverEvent, "alert")], conf)

    # cg.add_library("https://github.com/slimcdk/TMC-API", "3.10.3")
    cg.add_library("/workspaces/esphome/config/TMC-API", None)


def final_validate_config(config):
    tmc2209s = fv.full_config.get()[CONF_TMC2209]
    cg.add_define("TMC2209_NUM_COMPONENTS", len(tmc2209s))
    return config


FINAL_VALIDATE_SCHEMA = final_validate_config


@automation.register_action(
    "tmc2209.configure",
    TMC2209ConfigureAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TMC2209),
            cv.Optional(CONF_INVERSE_DIRECTION): cv.boolean,
            cv.Optional(CONF_MICROSTEPS, default=1): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 1)
            ),
            cv.Optional(CONF_RMS_CURRENT): cv.templatable(
                cv.All(cv.current, cv.positive_float)
            ),
            cv.Optional(CONF_RMS_CURRENT_HOLD_SCALE): cv.templatable(cv.percentage),
            cv.Optional(CONF_COOLSTEP_TCOOLTHRS): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
            cv.Optional(CONF_STALLGUARD_SGTHRS): cv.templatable(
                cv.int_range(min=0, max=2**8, max_included=True)
            ),
            cv.Optional(CONF_INTERPOLATION, default=True): cv.templatable(cv.boolean),
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
        template_ = yield cg.templatable(
            config[CONF_RMS_CURRENT_HOLD_SCALE], args, float
        )
        cg.add(var.set_rms_current_hold_scale(template_))

    # if CONF_HOLD_CURRENT_DELAY in config:
    #     template_ = yield cg.templatable(config[CONF_HOLD_CURRENT_DELAY], args, int)
    #     cg.add(var.ihold_irun_ihold_delay(template_))

    if CONF_MICROSTEPS in config:
        template_ = yield cg.templatable(config[CONF_MICROSTEPS], args, int)
        cg.add(var.set_microsteps(template_))

    if CONF_INTERPOLATION in config:
        template_ = yield cg.templatable(config[CONF_INTERPOLATION], args, int)
        cg.add(var.set_microstep_interpolation(template_))

    if CONF_COOLSTEP_TCOOLTHRS in config:
        template_ = yield cg.templatable(config[CONF_COOLSTEP_TCOOLTHRS], args, int)
        cg.add(var.set_coolstep_tcoolthrs(template_))

    if CONF_STALLGUARD_SGTHRS in config:
        template_ = yield cg.templatable(config[CONF_STALLGUARD_SGTHRS], args, int)
        cg.add(var.set_stallguard_sgthrs(template_))
    yield var
