import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.components import stepper
from . import tmc5240_ns, TMC5240, TMC5240_SCHEMA, register_tmc5240, CONF_TMC5240_ID


CODEOWNERS = ["@slimcdk"]

PLATFORM_STEPPER = "stepper"


TMC5240Stepper = tmc5240_ns.class_("TMC5240Stepper", TMC5240, stepper.Stepper)

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC5240_ID): cv.use_id(TMC5240Stepper),
    }
)

TMC5240_STEPPER_SCHEMA = cv.Schema({}).extend(TMC5240_SCHEMA, stepper.STEPPER_SCHEMA)


async def register_tmc5240_stepper(var, config):
    await register_tmc5240(var, config)
    await stepper.register_stepper(var, config)


# CONFIG_SCHEMA = cv.Schema(
#     {
#         cv.GenerateID(): cv.declare_id(TMC5240Stepper),
#     }
# ).extend(TMC5240_STEPPER_SCHEMA)


# def to_code(config):
#     pass
