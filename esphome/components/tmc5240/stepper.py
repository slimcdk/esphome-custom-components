from esphome import automation, pins
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import stepper, uart, spi
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_ADDRESS,
    CONF_TYPE,
)

CODEOWNERS = ["@slimcdk"]

CONF_SPI = "spi"
CONF_UART = "uart"

# AUTO_LOAD = [CONF_SPI, CONF_UART]

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
TMC5240Stepper = tmc5240_ns.class_("TMC5240Stepper", cg.Component, stepper.Stepper)

TMC5240ConfigureAction = tmc5240_ns.class_("TMC5240ConfigureAction", automation.Action)
TMC5240OnAlertTrigger = tmc5240_ns.class_("TMC5240OnAlertTrigger", automation.Trigger)

TMC5240SPIStepper = tmc5240_ns.class_(
    "TMC5240SPIStepper", TMC5240Stepper, spi.SPIDevice
)
TMC5240UARTStepper = tmc5240_ns.class_(
    "TMC5240UARTStepper", TMC5240Stepper, uart.UARTDevice
)

BASE_SCHEMA = (
    cv.Schema(
        {
            # cv.GenerateID(): cv.declare_id(TMC5240Stepper),
            cv.Required(CONF_ENN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIAG0_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_DIAG1_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_ON_ALERT): automation.validate_automation(
                # {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(TMC5240OnAlertTrigger)}
            ),
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(stepper.STEPPER_SCHEMA)
)


SPI_SCHEMA = BASE_SCHEMA.extend(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC5240SPIStepper),
        }
    ).extend(spi.spi_device_schema(cs_pin_required=True))
)

UART_SCHEMA = BASE_SCHEMA.extend(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC5240UARTStepper),
            cv.Optional(CONF_ADDRESS, default=0x00): cv.hex_uint8_t,
        }
    ).extend(uart.UART_DEVICE_SCHEMA)
)

CONFIG_SCHEMA = cv.typed_schema(
    {
        CONF_SPI: SPI_SCHEMA,
        CONF_UART: UART_SCHEMA,
    },
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

    cg.add_define("TMC5240_NUM_COMPONENTS", 255)
    cg.add_define("TMC5240_ENABLE_TMC_CACHE", 255)
    cg.add_define("TMC5240_CACHE", True)
    cg.add_library("https://github.com/slimcdk/TMC-API", "3.10.3")

    return var
