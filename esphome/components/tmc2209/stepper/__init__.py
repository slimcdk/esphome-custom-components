from esphome import automation, pins
from esphome.components import stepper

# from esphome.components.stepper import validate_speed
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID
)

from .. import (
    tmc_ns,
    TMC2209,
    TMC2209_SCHEMA,
    register_tmc2209,

    CONF_TMC2209_ID,
    CONF_INDEX_PIN,
    CONF_RSENSE,
    CONF_ON_MOTOR_STALL
)

CODEOWNERS = ["@slimcdk"]

TMC2209Stepper = tmc_ns.class_("TMC2209Stepper", cg.Component, stepper.Stepper, TMC2209)

TMC2209StepperMotorStallTrigger = tmc_ns.class_("TMC2209StepperMotorStallTrigger", automation.Trigger)


CONFIG_SCHEMA = (
    TMC2209_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TMC2209Stepper),
            cv.Required(CONF_INDEX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_RSENSE): cv.resistance,
            cv.Optional(CONF_ON_MOTOR_STALL): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        TMC2209StepperMotorStallTrigger
                    ),
                }
            ),
        },
    )
    .extend(stepper.STEPPER_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)
    await register_tmc2209(var, config)

    if CONF_INDEX_PIN in config:
        cg.add(var.set_index_pin(await cg.gpio_pin_expression(config[CONF_INDEX_PIN])))


TMC2209_DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC2209_ID): cv.use_id(TMC2209Stepper),
    }
)
