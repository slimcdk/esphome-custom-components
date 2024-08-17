from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import stepper
from esphome.const import CONF_ID

from .. import (
    tmc2209_ns,
    CONF_TMC2209,
    CONF_TMC2209_ID,
    TMC2209_CONFIG_SCHEMA,
    CONF_ENN_PIN,
    CONF_INDEX_PIN,
)

CODEOWNERS = ["@slimcdk"]
DEPENDENCIES = ["tmc2209", "stepper"]


TMC2209Stepper = tmc2209_ns.class_("TMC2209Stepper", cg.Component, stepper.Stepper)


# def validate_index_pin_required(config):

#     parent_id = config.get(CONF_TMC2209_ID)
#     tmc2209s = fv.full_config.get()[CONF_TMC2209]
#     print(parent_id, tmc2209s)
#     #

#     # if not parent_config or CONF_INDEX_PIN not in parent_config:
#     #     raise cv.Invalid(
#     #         f"The '{CONF_INDEX_PIN}' must be set in the parent TMC2209 component configuration when using the TMC2209Stepper child component."
#     #     )
#     return config


CONFIG_SCHEMA = cv.All(
    TMC2209_CONFIG_SCHEMA.extend(
        cv.Schema(
            {
                cv.GenerateID(): cv.declare_id(TMC2209Stepper),
                cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
            },
        )
    )
    .extend(stepper.STEPPER_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_parented(var, config[CONF_TMC2209_ID])
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)
    cg.add(var.set_enn_pin(await cg.gpio_pin_expression(config[CONF_ENN_PIN])))


def final_validate_config(config):
    # parent_id = config.get(CONF_TMC2209_ID)
    tmc2209s = fv.full_config.get()[CONF_TMC2209]

    if CONF_INDEX_PIN not in tmc2209s:
        raise cv.Invalid(
            f"The '{CONF_INDEX_PIN}' must be set in the parent TMC2209 component configuration when using the TMC2209Stepper child component."
        )

    return config


FINAL_VALIDATE_SCHEMA = final_validate_config
