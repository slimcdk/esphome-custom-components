import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.components import stepper
from . import tmc5241_ns, TMC5241, TMC5241_SCHEMA, register_tmc5241, CONF_TMC5241_ID


CODEOWNERS = ["@slimcdk"]

PLATFORM_STEPPER = "stepper"


TMC5241Stepper = tmc5241_ns.class_("TMC5241Stepper", TMC5241, stepper.Stepper)

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC5241_ID): cv.use_id(TMC5241Stepper),
    }
)

TMC5241_STEPPER_SCHEMA = cv.Schema({}).extend(TMC5241_SCHEMA, stepper.STEPPER_SCHEMA)


async def register_tmc5241_stepper(var, config):
    await register_tmc5241(var, config)
    await stepper.register_stepper(var, config)


# CONFIG_SCHEMA = cv.Schema(
#     {
#         cv.GenerateID(): cv.declare_id(TMC5241Stepper),
#     }
# ).extend(TMC5241_STEPPER_SCHEMA)


# def to_code(config):
#     pass
