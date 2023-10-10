from esphome import automation, pins
from esphome.components import uart, stepper
import esphome.config_validation as cv
import esphome.codegen as cg

from esphome.const import CONF_TRIGGER_ID, CONF_ID

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["uart"]

CONF_TMC2300_ID = "tmc2300_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_INDEX_PIN = "index_pin"

CONF_INVERSE_DIRECTION = "inverse_direction"
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

CONF_ON_FAULT_SIGNAL = "on_fault_signal"

tmc_ns = cg.esphome_ns.namespace("tmc")
TMC2300Stepper = tmc_ns.class_("TMC2300Stepper", cg.Component, stepper.Stepper, uart.UARTDevice)

TMC2300StepperConfigureAction = tmc_ns.class_("TMC2300StepperConfigureAction", automation.Action)
TMC2300StepperFaultSignalTrigger = tmc_ns.class_("TMC2300StepperFaultSignalTrigger", automation.Trigger)

CONFIG_SCHEMA = (
    stepper.STEPPER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TMC2300Stepper),
            cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_RSENSE): cv.resistance,
            cv.Optional(CONF_ON_FAULT_SIGNAL): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        TMC2300StepperFaultSignalTrigger
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

    for conf in config.get(CONF_ON_FAULT_SIGNAL, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.5.2")
