# from esphome import automation, pins
from esphome.components import sensor, uart
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID

CODEOWNERS = ["@slimcdk"]


freematics_ns = cg.esphome_ns.namespace("freematics")
FreematicsPlus = freematics_ns.class_(
    "FreematicsPlus",
    sensor.Sensor,
    cg.PollingComponent,
    uart.UARTDevice,
)


CONFIG_SCHEMA = (
    sensor.SENSOR_SCHEMA.extend(
        {
            cv.Required(CONF_ID): cv.declare_id(FreematicsPlus),
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    await uart.register_uart_device(var, config)

    cg.add_library("https://github.com/stanleyhuangyc/Freematics", None)
