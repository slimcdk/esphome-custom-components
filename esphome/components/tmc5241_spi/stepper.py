from esphome import pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.const import CONF_ID, CONF_PLATFORM
from esphome.components import spi
from esphome.components.tmc5241 import tmc5241_ns, CONF_DIAG1_PIN

from esphome.components.tmc5241.stepper import (
    PLATFORM_STEPPER,
    TMC5241Stepper,
    TMC5241_STEPPER_SCHEMA,
    register_tmc5241_stepper,
)


CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc5241"]

CONF_TMC5241_SPI = "tmc5241_spi"


TMC5241SPIStepper = tmc5241_ns.class_(
    "TMC5241SPIStepper", TMC5241Stepper, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC5241SPIStepper),
            cv.Optional(CONF_DIAG1_PIN): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(TMC5241_STEPPER_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await register_tmc5241_stepper(var, config)
    return var
