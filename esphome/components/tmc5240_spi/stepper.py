from esphome import pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.components import spi
from esphome.components.tmc5240 import tmc5240_ns, CONF_DIAG1_PIN

from esphome.components.tmc5240.stepper import (
    PLATFORM_STEPPER,
    TMC5240Stepper,
    TMC5240_STEPPER_SCHEMA,
    register_tmc5240_stepper,
)


CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc5240"]

CONF_TMC5240_SPI = "tmc5240_spi"


TMC5240SPIStepper = tmc5240_ns.class_(
    "TMC5240SPIStepper", TMC5240Stepper, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC5240SPIStepper),
            cv.Optional(CONF_DIAG1_PIN): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(TMC5240_STEPPER_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await register_tmc5240_stepper(var, config)
    return var
