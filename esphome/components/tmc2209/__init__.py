## Datasheet https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

from esphome import automation, pins
from esphome.components import uart
from esphome.automation import maybe_simple_id
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_ENABLE_PIN, CONF_TRIGGER_ID
from esphome.core import CORE, coroutine_with_priority

CODEOWNERS = ["@slimcdk"]

# MULTI_CONF = False

CONF_TMC2209_ID = "tmc2209_id"

CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"

CONF_VELOCITY = "velocity"
CONF_MICROSTEPS = "microsteps"
CONF_TCOOLTHRS = "tcool_threshold"
CONF_SGTHRS = "sg_threshold"
CONF_INVERSE_DIRECTION = "inverse_direction"
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

CONF_ON_MOTOR_STALL = "on_motor_stall"

tmc_ns = cg.esphome_ns.namespace("tmc")
TMC2209 = tmc_ns.class_("TMC2209", uart.UARTDevice)

TMC2209_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ADDRESS, default=0): cv.uint8_t,
        cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def setup_tmc2209_core_(tmc2209_var, config):
    await uart.register_uart_device(tmc2209_var, config)

    if CONF_ADDRESS in config:
        cg.add(tmc2209_var.set_address(config[CONF_ADDRESS]))

    if CONF_ENABLE_PIN in config:
        cg.add(tmc2209_var.set_enable_pin(await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])))

    if CONF_DIAG_PIN in config:
        cg.add(tmc2209_var.set_diag_pin(await cg.gpio_pin_expression(config[CONF_DIAG_PIN])))

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.5.1")


async def register_tmc2209(var, config):
    if not CORE.has_id(config[CONF_ID]):
        var = cg.Pvariable(config[CONF_ID], var)
    await setup_tmc2209_core_(var, config)


@coroutine_with_priority(100.0)
async def to_code(config):
    cg.add_global(tmc_ns.using)
