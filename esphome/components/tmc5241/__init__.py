from esphome import pins
import esphome.config_validation as cv
import esphome.codegen as cg


CODEOWNERS = ["@slimcdk"]

CONF_TMC5241 = "tmc5241"
CONF_TMC5241_ID = "tmc5241_id"

CONF_ENABLE = "enable"
CONF_DISABLE = "disable"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG0_PIN = "diag0_pin"
CONF_DIAG1_PIN = "diag1_pin"


CONF_ON_ALERT = "on_alert"


tmc5241_ns = cg.esphome_ns.namespace("tmc5241")
TMC5241 = tmc5241_ns.class_("TMC5241", cg.Component)


TMC5241_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ENN_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_DIAG0_PIN): pins.internal_gpio_input_pin_schema,
    },
).extend(cv.COMPONENT_SCHEMA)


async def register_tmc5241(var, config):
    await cg.register_component(var, config)

    enn_pin = config.get(CONF_ENN_PIN, None)
    diag0_pin = config.get(CONF_DIAG0_PIN, None)
    diag1_pin = config.get(CONF_DIAG1_PIN, None)

    if enn_pin is not None:
        cg.add(var.set_enn_pin(await cg.gpio_pin_expression(enn_pin)))

    if diag0_pin is not None:
        cg.add(var.set_diag0_pin(await cg.gpio_pin_expression(diag0_pin)))

    if diag1_pin is not None:
        cg.add(var.set_diag1_pin(await cg.gpio_pin_expression(diag1_pin)))

    # cg.add_library("https://github.com/slimcdk/TMC-API", "3.10.3")
    # cg.add_build_flag("-std=c++17")
    # cg.add_build_flag("-std=gnu++17")


# def final_validate_config(config):
#     tmc5241_steppers = [
#         stepper
#         for stepper in fv.full_config.get()[PLATFORM_STEPPER]
#         if stepper[CONF_PLATFORM] == CONF_TMC5241_SPI
#     ]
#     cg.add_define("TMC5241_NUM_SPI_COMPONENTS", len(tmc5241_steppers))


# FINAL_VALIDATE_SCHEMA = final_validate_config
