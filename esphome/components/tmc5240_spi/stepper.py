import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi, tmc5240, stepper
from esphome.const import CONF_ID

CODEOWNERS = ["@slimcdk"]

AUTO_LOAD = ["tmc5240"]
DEPENDENCIES = ["spi"]

tmc5240_spi_ns = cg.esphome_ns.namespace("tmc5240_spi")
TMC5240SPIStepper = tmc5240_spi_ns.class_(
    "TMC5240SPIStepper", cg.Component, stepper.Stepper, spi.SPIDevice
)


CONFIG_SCHEMA = tmc5240.CONFIG_SCHEMA_BASE.extend(
    spi.spi_device_schema(cs_pin_required=True)
).extend({cv.GenerateID(): cv.declare_id(TMC5240SPIStepper)})


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await tmc5240.to_code_base(var, config)
    await spi.register_spi_device(var, config)
