import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import tmc5240, spi
from esphome.const import (
    CONF_ID,
)

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc5240"]
DEPENDENCIES = ["spi"]

# CONF_TMC5240_ID = "tmc5240_spi_id"

TMC5240SPI = tmc5240.tmc5240_ns.class_("TMC5240SPI", tmc5240.TMC5240, spi.SPIDevice)


CONFIG_SCHEMA = cv.All(
    tmc5240.TMC5240_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TMC5240SPI)
        }
    )
    .extend(spi.spi_device_schema()),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await tmc5240.setup_tmc5240_base(var, config)

FINAL_VALIDATE_SCHEMA = tmc5240.final_validate_config
