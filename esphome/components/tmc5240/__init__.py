from esphome import automation, pins
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import stepper, uart, spi
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_SPI_ID,
    CONF_UART_ID,
    CONF_ADDRESS,
)

CODEOWNERS = ["@slimcdk"]


CONF_TMC5240 = "tmc5240"
CONF_TMC5240_ID = "tmc5240_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG0_PIN = "diag0_pin"
CONF_DIAG1_PIN = "diag1_pin"

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

CONF_ON_ALERT = "on_alert"

tmc5240_ns = cg.esphome_ns.namespace("tmc5240")
TMC5240Stepper = tmc5240_ns.class_("TMC5240Stepper", cg.Component, stepper.Stepper)

# TMC5240ConfigureAction = tmc5240_ns.class_("TMC5240ConfigureAction", automation.Action)
# TMC2209OnAlertTrigger = tmc5240_ns.class_("TMC2209OnAlertTrigger", automation.Trigger)


CONFIG_SCHEMA_BASE = (
    cv.Schema(
        {
            cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIAG0_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_DIAG1_PIN): pins.gpio_input_pin_schema,
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(stepper.STEPPER_SCHEMA)
)


async def to_code_base(var, config):

    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)

    cg.add(var.set_enn_pin(await cg.gpio_pin_expression(config[CONF_ENN_PIN])))
    cg.add(var.set_diag0_pin(await cg.gpio_pin_expression(config[CONF_DIAG0_PIN])))
    cg.add(var.set_diag1_pin(await cg.gpio_pin_expression(config[CONF_DIAG1_PIN])))

    for conf in config.get(CONF_ON_ALERT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_define("TMC5240_NUM_COMPONENTS", 255)
    cg.add_define("TMC5240_ENABLE_TMC_CACHE", 255)
    cg.add_define("TMC5240_CACHE", True)
    cg.add_library("https://github.com/slimcdk/TMC-API", "3.10.3")

    return var
