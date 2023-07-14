import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor
from esphome.const import CONF_PIN
from .. import gpio_ns

CONF_USE_INTERRUPT = "use_interrupt"

GPIOBinarySensor = gpio_ns.class_(
    "GPIOBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(GPIOBinarySensor)
    .extend(
        {
            cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_USE_INTERRUPT, default=False): bool,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    if config[CONF_USE_INTERRUPT]:
        cg.add_define("USE_INTERRUPT")

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
