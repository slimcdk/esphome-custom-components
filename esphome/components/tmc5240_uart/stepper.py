from esphome import pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.components import uart
from esphome.components.tmc5240 import tmc5240_ns, CONF_DIAG0_PIN

from esphome.components.tmc5240.stepper import (
    TMC5240Stepper,
    TMC5240_STEPPER_SCHEMA,
    register_tmc5240_stepper,
)


CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc5240"]

CONF_TMC5240_UART = "tmc5240_uart"


TMC5240UARTStepper = tmc5240_ns.class_(
    "TMC5240UARTStepper", TMC5240Stepper, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC5240UARTStepper),
            cv.Optional(CONF_DIAG0_PIN): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(TMC5240_STEPPER_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await uart.register_uart_device(var, config)
    await register_tmc5240_stepper(var, config)

    return var


FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    CONF_TMC5240_UART, require_rx=True, require_tx=True
)
