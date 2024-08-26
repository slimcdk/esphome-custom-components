from esphome import automation, pins
import esphome.config_validation as cv
import esphome.final_validate as fv
import esphome.codegen as cg
from esphome.components import stepper, uart, spi
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_TYPE,
    CONF_PLATFORM,
)


CODEOWNERS = ["@slimcdk"]

CONF_SPI = "spi"
CONF_UART = "uart"

CONF_STEPPER = "stepper"
CONF_TMC5240 = "tmc5240"
CONF_TMC5240_ID = "tmc5240_id"

CONF_ENN_PIN = "enn_pin"
CONF_DIAG0_PIN = "diag0_pin"
CONF_DIAG1_PIN = "diag1_pin"

CONF_INVERSE_DIRECTION = "inverse_direction"
CONF_VELOCITY = "velocity"
CONF_VACTUAL = "vactual"
CONF_MICROSTEPS = "microsteps"
CONF_COOLSTEP_TCOOLTHRS = "coolstep_tcoolthrs"
CONF_STALLGUARD_SGTHRS = "stallguard_sgthrs"
CONF_RMS_CURRENT = "rms_current"
CONF_RMS_CURRENT_HOLD_SCALE = "rms_current_hold_scale"
CONF_HOLD_CURRENT_DELAY = "hold_current_delay"
CONF_POWER_DOWN_DELAY = "power_down_delay"
CONF_TSTEP = "tstep"
CONF_TPWMTHRS = "tpwmthrs"
CONF_RSENSE = "rsense"
CONF_INTERNAL_RSENSE = "internal_rsense"

CONF_ON_ALERT = "on_alert"


tmc5240_ns = cg.esphome_ns.namespace("tmc5240")
Stepper = tmc5240_ns.class_("TMC5240Stepper", cg.Component, stepper.Stepper)
SPIStepper = tmc5240_ns.class_("TMC5240SPIStepper", Stepper, spi.SPIDevice)
UARTStepper = tmc5240_ns.class_("TMC5240UARTStepper", Stepper, uart.UARTDevice)

ConfigureAction = tmc5240_ns.class_("TMC5240ConfigureAction", automation.Action)
OnAlertTrigger = tmc5240_ns.class_("TMC5240OnAlertTrigger", automation.Trigger)


BASE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_DIAG0_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_ON_ALERT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnAlertTrigger),
            }
        ),
    },
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.typed_schema(
    {
        CONF_SPI: cv.Schema(
            {
                cv.GenerateID(): cv.declare_id(SPIStepper),
                cv.Optional(CONF_DIAG1_PIN): pins.gpio_input_pin_schema,
            }
        ).extend(
            BASE_SCHEMA,
            spi.spi_device_schema(cs_pin_required=True),
            stepper.STEPPER_SCHEMA,
        ),
        CONF_UART: cv.Schema(
            {
                cv.GenerateID(): cv.declare_id(UARTStepper),
                cv.Required(CONF_DIAG1_PIN): pins.gpio_pin_schema,
                cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
            }
        ).extend(
            BASE_SCHEMA,
            uart.UART_DEVICE_SCHEMA,
            stepper.STEPPER_SCHEMA,
        ),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await stepper.register_stepper(var, config)

    if CONF_TYPE in config and config[CONF_TYPE] == CONF_SPI:
        cg.add_define("TMC5240_USE_SPI")
        await spi.register_spi_device(var, config)

    elif CONF_TYPE in config and config[CONF_TYPE] == CONF_UART:
        cg.add_define("TMC5240_USE_UART")
        cg.add(var.set_uart_address(config[CONF_ADDRESS]))
        await uart.register_uart_device(var, config)

    cg.add(var.set_enn_pin(await cg.gpio_pin_expression(config[CONF_ENN_PIN])))
    cg.add(var.set_diag0_pin(await cg.gpio_pin_expression(config[CONF_DIAG0_PIN])))
    cg.add(var.set_diag1_pin(await cg.gpio_pin_expression(config[CONF_DIAG1_PIN])))

    for conf in config.get(CONF_ON_ALERT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_library("https://github.com/slimcdk/TMC-API", "3.10.3")

    return var


def final_validate_config(config):

    if CONF_TYPE in config and config[CONF_TYPE] == CONF_UART:
        uart.final_validate_device_schema("tmc5240", require_rx=True, require_tx=True)(
            config
        )

    steppers = fv.full_config.get()[CONF_STEPPER]
    tmc5240_steppers = [
        stepper for stepper in steppers if stepper[CONF_PLATFORM] == CONF_TMC5240
    ]
    cg.add_define("TMC5240_NUM_COMPONENTS", len(tmc5240_steppers))


FINAL_VALIDATE_SCHEMA = final_validate_config


@automation.register_action(
    "tmc5240.configure",
    ConfigureAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Stepper),
            cv.Optional(CONF_INVERSE_DIRECTION): cv.boolean,
            cv.Optional(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(256, 128, 64, 32, 16, 8, 4, 2, 0)
            ),
            cv.Optional(CONF_RMS_CURRENT): cv.templatable(
                cv.All(cv.current, cv.positive_float)
            ),
            cv.Optional(CONF_COOLSTEP_TCOOLTHRS): cv.templatable(
                cv.int_range(min=0, max=2**20, max_included=False)
            ),
            cv.Optional(CONF_STALLGUARD_SGTHRS): cv.templatable(
                cv.int_range(min=0, max=2**8, max_included=True)
            ),
        }
    ),
)
def tmc5240_configure_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    yield cg.register_parented(var, config[CONF_ID])

    if CONF_INVERSE_DIRECTION in config:
        template_ = yield cg.templatable(config[CONF_INVERSE_DIRECTION], args, bool)
        cg.add(var.set_inverse_direction(template_))

    if CONF_MICROSTEPS in config:
        template_ = yield cg.templatable(config[CONF_MICROSTEPS], args, int)
        cg.add(var.set_microsteps(template_))

    if CONF_RMS_CURRENT in config:
        template_ = yield cg.templatable(config[CONF_RMS_CURRENT], args, float)
        cg.add(var.set_rms_current(template_))

    if CONF_MICROSTEPS in config:
        template_ = yield cg.templatable(config[CONF_MICROSTEPS], args, int)
        cg.add(var.set_microsteps(template_))

    if CONF_COOLSTEP_TCOOLTHRS in config:
        template_ = yield cg.templatable(config[CONF_COOLSTEP_TCOOLTHRS], args, int)
        cg.add(var.set_coolstep_tcoolthrs(template_))

    if CONF_STALLGUARD_SGTHRS in config:
        template_ = yield cg.templatable(config[CONF_STALLGUARD_SGTHRS], args, int)
        cg.add(var.set_stallguard_sgthrs(template_))
    yield var


TMC5240_ACTION_SCHEMA = automation.maybe_simple_id(
    {cv.Required(CONF_ID): cv.use_id(Stepper)}
)


# @automation.register_action("tmc5240.stop", StopAction, TMC5240_ACTION_SCHEMA)
# def tmc5240_stop_to_code(config, action_id, template_arg, args):
#     var = cg.new_Pvariable(action_id, template_arg)
#     yield cg.register_parented(var, config[CONF_ID])
#     yield var
