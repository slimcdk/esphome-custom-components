from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import stepper
from esphome.const import CONF_ID


from .. import (
    tmc2209_ns,
    CONF_TMC2209_ID,
    TMC2209_CONFIG_SCHEMA,
    CONF_ENN_PIN,
    final_validate_device_schema,
)

CODEOWNERS = ["@slimcdk"]

DEPENDENCIES = ["tmc2209", "stepper"]

TMC2209Stepper = tmc2209_ns.class_("TMC2209Stepper", cg.Component, stepper.Stepper)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC2209Stepper),
            cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
        },
    )
    .extend(TMC2209_CONFIG_SCHEMA)
    .extend(stepper.STEPPER_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_parented(var, config[CONF_TMC2209_ID])
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)
    cg.add(var.set_enn_pin(await cg.gpio_pin_expression(config[CONF_ENN_PIN])))


FINAL_VALIDATE_SCHEMA = final_validate_device_schema(
    "tmc2209.stepper", require_index=True
)
