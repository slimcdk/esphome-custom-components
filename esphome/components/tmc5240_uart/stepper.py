import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, tmc5240, stepper
from esphome.const import CONF_ID, CONF_ADDRESS

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc5240"]
DEPENDENCIES = ["uart"]

tmc5240_uart_ns = cg.esphome_ns.namespace("tmc5240_uart")
TMC5240UARTStepper = tmc5240_uart_ns.class_(
    "TMC5240UARTStepper", cg.Component, stepper.Stepper, uart.UARTDevice
)


CONFIG_SCHEMA = tmc5240.CONFIG_SCHEMA_BASE.extend(uart.UART_DEVICE_SCHEMA).extend(
    {
        cv.GenerateID(): cv.declare_id(TMC5240UARTStepper),
        cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_ADDRESS])
    await tmc5240.to_code_base(var, config)
    await uart.register_uart_device(var, config)
