# TMC2209

ESPHome component to control a stepper motor using an ADI (formerly Trinamic) TMC2209 stepper motor driver over UART. Technical information can be found in the [section 1.2.1][datasheet].

This implementation contains multiple parts: a base component that facilitates serial communication with and feedback from the TMC2209 driver, and a stepper component that allows control of the motor over serial or via step/dir.

> [!IMPORTANT]
*Only a single `tmc2209` instance (device) per UART config is currently supported by ESPHome. Multiple drivers require multiple UART connections.*

# Table of contents

- [Config](#config)
  - [UART Setup](#uart-bus-configuration)
  - [TMC2209 Base Configuration](#base-configuration)
  - [Stepper Control](#the-stepper-can-be-controlled-in-two-ways)
    - [Serial (UART)](#using-serial-uart)
    - [Pulse Train](#using-traditional-stepping-pulses-and-direction)
- [Automation](#automation)
  - [Alert Events](#current-supported-alert-events)
- [Actions](#actions)
  - [`tmc2209.configure`](#tmc2209configure-action)
- [Driver sensors](#sensors)
- [Example Config](#example-config)
- [Advanced](#advanced)
- [Wiring](#wiring)
  - [For UART control](#uart-control)
  - [For pulse train control](#pulse-train-control)
- [Resources](#resources)
- [Troubleshooting](#troubleshooting)

## Config

Import the component(s).
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ tmc2209, stepper, stepdir ]
```
---

Configuration of [UART Bus][uart-component].

> [!IMPORTANT]
**TX and RX must be provided**

> [!CAUTION]
**A lot is happening over serial and low baud rates might cause warnings about the component taking too long. Use something like 115200 or higher.**

```yaml
uart:
  tx_pin: REPLACEME
  rx_pin: REPLACEME
  baud_rate: 115200 # 9600 -> 500k
```
> *TMC2209 will auto-detect baud rates from 9600 to 500k with the internal clock/oscillator. An external clock/oscillator is needed for baud rates higher than 500k.*
---

Base configuration that facilitates fundamental interaction with the driver like serial communication, event handling and basic configuration, for a generic setup with ESPHome.

```yaml
tmc2209:
  id: driver
  address: 0x00
  diag_pin: REPLACEME # highly recommended to set
  rsense: 110 mOhm    # highly recommended to set
  clock_frequency: 12MHz
  overtemperature:
    prewarning: 120C
    shutdown: 143C
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the component so that you can reference it from elsewhere.

* `diag_pin` (*Optional*, [Input Pin Schema][config-pin]): Driver error signaling from the driver.
  >If not defined, the less reliable detection over UART will be used instead.

* `address` (*Optional*, hex): UART address of the IC. Configured by setting MS1_AD0 or MS2_AD1 high or low. Default is `0x00`.

* `rsense` (*Optional*, resistance): Motor current sense resistors. Varies from ~75 to 1000 mOhm. Consult [section 8][datasheet] for a lookup table.

* `clock_frequency` (*Optional*, frequency): Timing reference for all functionalities of the driver. Defaults to 12MHz, which all drivers are factory calibrated to. Only set if using external clock.

* `overtemperature` (*Optional*, dict): Limits for warning and shutdown temperatures. Default is OTP which is factory configuration of 120C for prewarning and 143C for overtemperature (shutdown).
  * `prewarning` (**Required**, temperature): At which a prewarning for overtemperature is signaled.
  * `shutdown` (**Required**, temperature): At which the driver disables the output. Can be reenabled once temperature is below prewarning and `ENN` has been toggled.

  Valid combinations are:
    * `prewarning=120C` and `shutdown=143C`.
    * `prewarning=120C` and `shutdown=150C`.
    * `prewarning=143C` and `shutdown=150C`.
    * `prewarning=143C` and `shutdown=157C`.

>[!WARNING]
*Configuring `overtemperature` has no effect at the moment as there is an issue with updating the register field on the driver. It will however still monitor temperatures and emit alert events and the driver will also shutdown when temperature is too high*

---
### The stepper can be controlled in two ways
> [!CAUTION]
**Don't configure multiple stepper components for same physical driver!**

#### Using serial (UART)
Accuracy is slightly reduced in favor of tight timings and high-frequency stepping pulses. Pulse generation is unaffected by ESPHome's handling of other components or main thread execution. This means that the host microcontroller (e.g., ESP32) running ESPHome doesn't provide the step generation, but it is handled internally by the driver. Highly recommended for use with high microstep interpolation or true silent operation.
>*The provided accuracy is often precise enough, but it depends on speeds and how often the component can write to the driver.*

Relevant info can be found in [section 1.3][datasheet].

```yaml
stepper:
  - platform: tmc2209
    id: motor
    enn_pin: REPLACEME
    index_pin: REPLACEME
    max_speed: 500 steps/s
    acceleration: 1000 steps/s^2 # optional
    deceleration: 1000 steps/s^2 # optional
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the stepper so that you can control it.

* `enn_pin` (**Required**, [Output Pin Schema][config-pin]): Enable not input pin for the driver. No need for manual inverted config as inverted logic is handled internally.

* `index_pin` (**Required**, [Input Pin Schema][config-pin]): Serves as stepping feedback from the internal step pulse generator.

* All other from [Base Stepper Component][base-stepper-component]


#### Using traditional stepping pulses and direction
Stepping pulses are handled by the main thread but utilize [increased execution frequency functionality][highfrequencylooprequester] to generate pulses as fast as possible. Pulses are therefore limited to whenever the ESP can generate a pulse, and any timing inconsistencies become audible when the motor runs.
>*More components take up more resources slowing the main thread.*

Example config of [stepdir](../stepdir/) for a TMC2209 driver.
```yaml
stepper:
  - platform: stepdir
    id: motor
    step_pin: REPLACEME
    dir_pin: REPLACEME
    max_speed: 500 steps/s

    # optional
    sleep_pin:
      number: REPLACEME
      inverted: true
    acceleration: 1000 steps/s^2
    deceleration: 1000 steps/s^2
```
> *Inverted sleep pin is required for TMC2209*


## Automation

### `on_alert`
An alert event is fired whenever a driver warning or error is detected. For instance when the motor stalls. This event can be used for sensorless homing. Works both with control over serial (UART) and with stepping and direction pulses.
```yaml
tmc2209:
  id: driver
  ...
  on_alert:
    - if:
        condition:
          lambda: return alert == tmc2209::STALLED;
        then:
          - logger.log: "Motor stalled!"
          - stepper.stop: motor
```
> <small>[All events in an example config](#alert-events)</small>



#### Current supported alert events

Most alerts is signaling that the driver is in a given state. The majority of alert events also has a `CLEARED` or similar counterpart signaling that the driver is now not in the given state anymore.

  * `DIAG_TRIGGERED` DIAG output is triggered. Primarily driver errors.
  * `STALLED` StallGuard result crossed StallGuard threshold and motor is considered stalled.
  * `OVERTEMPERATURE_PREWARNING` | `OVERTEMPERATURE_PREWARNING_CLEARED` Driver is warning about increasing temperature.
  * `OVERTEMPERATURE` | `OVERTEMPERATURE_CLEARED` Driver is at critial high temperature and is shutting down.
  * `TEMPERATURE_ABOVE_120C` | `TEMPERATURE_BELOW_120C` Temperature is higher or lower than 120C.
  * `TEMPERATURE_ABOVE_143C` | `TEMPERATURE_BELOW_143C` Temperature is higher or lower than 143C.
  * `TEMPERATURE_ABOVE_150C` | `TEMPERATURE_BELOW_150C` Temperature is higher or lower than 150C.
  * `TEMPERATURE_ABOVE_157C` | `TEMPERATURE_BELOW_157C` Temperature is higher or lower than 157C.
  * `A_OPEN_LOAD` | `A_OPEN_LOAD_CLEARED` Open load indicator phase A.
  * `B_OPEN_LOAD` | `B_OPEN_LOAD_CLEARED` Open load indicator phase B.
  * `A_LOW_SIDE_SHORT` | `A_LOW_SIDE_SHORT_CLEARED` Low side short indicator phase A.
  * `B_LOW_SIDE_SHORT` | `B_LOW_SIDE_SHORT_CLEARED` Low side short indicator phase B.
  * `A_GROUND_SHORT` | `A_GROUND_SHORT_CLEARED` Short to ground indicator phase A.
  * `B_GROUND_SHORT` | `B_GROUND_SHORT_CLEARED` Short to ground indicator phase B.

>[!IMPORTANT]
*`STALLED` is the event you would want for sensorless homing*


## Actions

### `tmc2209.configure` Action
Example of configuring the driver. For instance on boot.
```yaml
esphome:
  ...
  on_boot:
    - tmc2209.configure:
        id: driver
        microsteps: 4
        coolstep_tcoolthrs: 400
        stallguard_sgthrs: 75
        rms_current: 800mA
```

* `id` (**Required**, ID): Reference to the stepper tmc2209 (base, not stepper) component.

* `microsteps` (*Optional*, int, [templatable][config-templatable]): Microstepping. Possible values are `1`, `2`, `4`, `8`, `16`, `32`, `64`, `128`, `256`.

* `inverse_direction` (*Optional*, bool, [templatable][config-templatable]): Inverse the rotational direction.

* `coolstep_tcoolthrs` (*Optional*, int, [templatable][config-templatable]): Value for the COOLSTEP TCOOL threshold.

* `stallguard_sgthrs` (*Optional*, int, [templatable][config-templatable]): Value for the StallGuard2 threshold.

* `interpolation` (*Optional*, bool, [templatable][config-templatable]): The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for the smoothest motor operation.

* `rms_current` (*Optional*, current, [templatable][config-templatable]): RMS current setting according to [section 9][datasheet].


### Sensors

Some metrics from the driver is exposed as a ready-to-use sensor component.

```yaml
sensor:
  - platform: tmc2209
    type: stallguard_result
    name: Driver stallguard
    update_interval: 1s

  - platform: tmc2209
    type: motor_load
    name: Motor load
    update_interval: 100ms
```
* `tmc2209_id` (*Optional*, [ID][config-id]): Manually specify the ID of the `tmc2209` you want to use this sensor.

* `type` (**Required**):
  * `stallguard_result` Stator angle shift detected by the driver.
  * `motor_load` Percentage off stall calculated from StallGuard result and set StallGuard threshold. 100% = stalled

* All other from [Sensor][base-sensor-component]


## Example config
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [tmc2209, stepper]

# esp32 or esp8266 config..

wifi:
  ssid: !secret WIFI_SSID
  password: !secret WIFI_PASSWORD

esphome:
  name: actuator
  on_boot:
    - tmc2209.configure:
        id: driver
        coolstep_tcoolthrs: 400
        stallguard_sgthrs: 50
        microsteps: 2
        interpolation: true
        rms_current: 800mA

uart:
  tx_pin: 27
  rx_pin: 26
  baud_rate: 115200

tmc2209:
  id: driver
  diag_pin: 13
  rsense: 110 mOhm
  on_alert:
    - if:
        condition:
          lambda: return alert == tmc2209::STALLED;
        then:
          - logger.log: "Motor stalled!"
          - stepper.stop: motor

stepper:
  - platform: tmc2209
    id: motor
    tmc2209_id: driver
    enn_pin: 14
    index_pin: 12
    max_speed: 500 steps/s
    acceleration: 1000 steps/s^2
    deceleration: 1000 steps/s^2

button:
  - platform: template
    name: Stop
    on_press:
      - stepper.stop: motor

  - platform: template
    name: 1000 Steps forward
    on_press:
      - stepper.set_target:
          id: motor
          target: !lambda return id(motor).current_position +1000;

  - platform: template
    name: 1000 Steps backward
    on_press:
      - stepper.set_target:
          id: motor
          target: !lambda return id(motor).current_position -1000;

number:
  - platform: template
    name: Target position
    min_value: -100000
    max_value: 100000
    step: 100
    lambda: return id(motor)->current_position;
    update_interval: 1s
    set_action:
      - stepper.set_target:
          id: motor
          target: !lambda "return x;"

sensor:
  - platform: tmc2209
    type: motor_load
    name: Motor load
    update_interval: 100ms
```

Output of the above configuration.
```console
...
[00:00:00][C][uart.idf:159]: UART Bus 1:
[00:00:00][C][uart.idf:160]:   TX Pin: GPIO27
[00:00:00][C][uart.idf:161]:   RX Pin: GPIO26
[00:00:00][C][uart.idf:163]:   RX Buffer Size: 256
[00:00:00][C][uart.idf:165]:   Baud Rate: 115200 baud
[00:00:00][C][uart.idf:166]:   Data Bits: 8
[00:00:00][C][uart.idf:167]:   Parity: NONE
[00:00:00][C][uart.idf:168]:   Stop bits: 1
[00:00:00][C][tmc2209:017]: TMC2209:
[00:00:00][C][tmc2209:019]:   DIAG Pin: GPIO13
[00:00:00][C][tmc2209:020]:   RSense: 0.11 Ohm (External)
[00:00:00][C][tmc2209:021]:   Address: 0x00
[00:00:00][C][tmc2209:022]:   Clock frequency: 12000000 Hz
[00:00:00][C][tmc2209:028]:   Detected IC version: 0x21
[00:00:00][C][tmc2209:039]:   Overtemperature: prewarning = 120C | shutdown = 143C
[00:00:00][C][tmc2209.stepper:011]: TMC2209 Stepper:
[00:00:00][C][tmc2209.stepper:012]:   ENN Pin: GPIO14
[00:00:00][C][tmc2209.stepper:013]:   INDEX Pin: GPIO12
[00:00:00][C][tmc2209.stepper:014]:   Acceleration: 1000 steps/s^2
[00:00:00][C][tmc2209.stepper:014]:   Deceleration: 1000 steps/s^2
[00:00:00][C][tmc2209.stepper:014]:   Max Speed: 500 steps/s
...
```



### Advanced

Writing to and reading from registers and register fields from the driver can easily be done with the help of preexisting [helper definitions][tmcapi-tmc2209-hwa] from the underlying TMC-API. A description of the register map can be found under [section 5][datasheet].
> [!IMPORTANT]
**The `tmc2209` base component holds a mirror in memory of the values written to the driver. This means write-only registers can still be read with below methods provided they have been written already.**
>
>*Definitions ending in `_MASK` or `_SHIFT` should not be used.*

The `tmc2209` base component exposes four methods:
* `void write_register(uint8_t address, int32_t value)` write/overwrite an entire register.
* `int32_t read_register(uint8_t address)` read an entire registers.
* `void write_field(RegisterField field, uint32_t value)` write/overwrite a register field.
* `uint32_t read_field(RegisterField field)` read a register field.

> [!CAUTION]
> Overwriting some registers may cause instability in the ESPHome component.


Example usage in lambdas
```yaml
sensor:

    // Read stallguard result (register) into a sensor
  - platform: template
    name: Stallguard result
    lambda: return id(driver)->read_register(TMC2209_SG_RESULT);

    // Read microstep selection index into a sensor. This is a binary exponent like 0,1,2,3,... and microsteps can be calculated like 2**<exponent>
  - platform: template
    name: Microstep selection index
    lambda: return id(driver)->read_field(TMC2209_MRES_FIELD);



number:

    // Write value to stallguard threshold register
  - platform: template
    name: Stallguard threshold
    update_interval: 1s
    min_value: 0
    max_value: 255
    step: 5
    lambda: return id(driver)->read_register(TMC2209_SGTHRS);
    set_action:
      - lambda: id(driver)->write_register(TMC2209_SGTHRS, x);



button:

    // Write value 3 to MRES register field. 2**3 = 8
  - platform: template
    name: Set microstepping to 8
    on_press:
      - lambda: id(driver)->write_field(TMC2209_MRES_FIELD, 3);

```


## Wiring
Guides to wire ESPHome supported MCU to a TMC2209 driver for either only UART control or pulse train control.

### UART Control

Wiring for [UART control](#using-serial-uart). `DIAG` is optional but recommended for reliability.

<img src="./docs/uart-wiring.svg" alt="UART wiring" width="100%" />


### Pulse train control
Wiring for [Pulse Train control](#using-traditional-stepping-pulses-and-direction).

<img src="./docs/sd-wiring.svg" alt="STEP/DIR wiring" width="100%" />


> [!IMPORTANT]
> Most drivers come as breakout modules and connections can often be labeled slightly differently. `PDN_UART` was often not labeled, as serial communication was rarly used in the early days, but is apparent on nearly all new modules.


#### Examples of modules
<p align="center">
  <img src="./docs/trinamic-bob-module.jpg" alt="Trinamic BOB" width="19%" />
  <img src="./docs/silentstepstick-module.jpg" alt="SilentStepStick" width="19%" />
  <img src="./docs/bigtreetech-module.webp" alt="BigTreeTech" width="19%" />
  <img src="./docs/fysetc-module.webp" alt="Fysetc" width="19%" />
  <img src="./docs/grobo-module.jpg" alt="GRobotronics" width="19%" />
</p>


## Resources
* https://esphome.io/components/uart
* https://esphome.io/components/stepper
* https://github.com/slimcdk/esphome-custom-components/tree/master/esphome/components/stepdir


## Troubleshooting

#### Unable to read IC version. Is the driver powered and wired correctly?
1. Make sure UART is correctly wired and the 1k Ohm resistor is placed correctly.
2. Make sure the driver is power on VM / VS (motor supply voltage). Must be between 4.75 and 29V.

#### Detected unknown IC version: 0x??
First generation of TMC2209s have version `0x21`. There is only a single version released as of Q3 2024. If you are seeing version `0x20` that means you have a TMC2208 which is not supported by this component.

#### Reading from UART timed out at byte 0!
Poor signal integrity can cause instability in the UART connection. The component doesn't retry writing/reading if a reading failed. Make sure the connection is reliable for best performance. Try lower baud rates if these only appear occasionally.

#### Driver makes "sizzling" noise
Activation of the driver is delegated to the stepper component to perform. Long wires connected to ENN might pick up interference causing the driver to make a sizzling noise if left floating. A stepper configuration is needed for handling ENN.

## TODOs
* Learning resources on how to tune StallGuard etc.
* Driver error detection on the DIAG pin or UART and event broadcasting.
* Write default register to driver.
* Reconfigure driver if driver was power cycled.
* Driver warning and error trigger events.
* Handle changes written directly to the driver.
* Write default register on start?
* OTTRIM not setting or reading properly


## MISC

### Alert events
All alert events configured for easy copy-pasta.
```yaml
tmc2209:
  id: driver
  ...
  on_alert:
    - if:
        condition:
          lambda: return alert == tmc2209::DIAG_TRIGGERED;
        then:
          - logger.log: DIAG_TRIGGERED
    - if:
        condition:
          lambda: return alert == tmc2209::STALLED;
        then:
          - logger.log: STALLED
          - stepper.stop: motor
    - if:
        condition:
          lambda: return alert == tmc2209::OVERTEMPERATURE_PREWARNING;
        then:
          - logger.log: OVERTEMPERATURE_PREWARNING
    - if:
        condition:
          lambda: return alert == tmc2209::OVERTEMPERATURE_PREWARNING_CLEARED;
        then:
          - logger.log: OVERTEMPERATURE_PREWARNING_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::OVERTEMPERATURE;
        then:
          - logger.log: OVERTEMPERATURE_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_ABOVE_120C;
        then:
          - logger.log: TEMPERATURE_ABOVE_120C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_BELOW_120C;
        then:
          - logger.log: TEMPERATURE_BELOW_120C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_ABOVE_143C;
        then:
          - logger.log: TEMPERATURE_ABOVE_143C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_BELOW_143C;
        then:
          - logger.log: TEMPERATURE_BELOW_143C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_ABOVE_150C;
        then:
          - logger.log: TEMPERATURE_ABOVE_150C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_BELOW_150C;
        then:
          - logger.log: TEMPERATURE_BELOW_150C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_ABOVE_157C;
        then:
          - logger.log: TEMPERATURE_ABOVE_157C
    - if:
        condition:
          lambda: return alert == tmc2209::TEMPERATURE_BELOW_157C;
        then:
          - logger.log: TEMPERATURE_BELOW_157C
    - if:
        condition:
          lambda: return alert == tmc2209::A_OPEN_LOAD;
        then:
          - logger.log: A_OPEN_LOAD
    - if:
        condition:
          lambda: return alert == tmc2209::A_OPEN_LOAD_CLEARED;
        then:
          - logger.log: A_OPEN_LOAD_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::B_OPEN_LOAD;
        then:
          - logger.log: B_OPEN_LOAD
    - if:
        condition:
          lambda: return alert == tmc2209::B_OPEN_LOAD_CLEARED;
        then:
          - logger.log: B_OPEN_LOAD_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::A_OPEN_LOAD;
        then:
          - logger.log: A_OPEN_LOAD
    - if:
        condition:
          lambda: return alert == tmc2209::A_OPEN_LOAD_CLEARED;
        then:
          - logger.log: A_OPEN_LOAD_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::B_OPEN_LOAD;
        then:
          - logger.log: B_OPEN_LOAD
    - if:
        condition:
          lambda: return alert == tmc2209::B_OPEN_LOAD_CLEARED;
        then:
          - logger.log: B_OPEN_LOAD_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::A_LOW_SIDE_SHORT;
        then:
          - logger.log: A_LOW_SIDE_SHORT
    - if:
        condition:
          lambda: return alert == tmc2209::A_LOW_SIDE_SHORT_CLEARED;
        then:
          - logger.log: A_LOW_SIDE_SHORT_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::B_LOW_SIDE_SHORT;
        then:
          - logger.log: B_LOW_SIDE_SHORT
    - if:
        condition:
          lambda: return alert == tmc2209::B_LOW_SIDE_SHORT_CLEARED;
        then:
          - logger.log: B_LOW_SIDE_SHORT_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::A_GROUND_SHORT_CLEARED;
        then:
          - logger.log: A_GROUND_SHORT_CLEARED
    - if:
        condition:
          lambda: return alert == tmc2209::B_GROUND_SHORT;
        then:
          - logger.log: B_GROUND_SHORT
    - if:
        condition:
          lambda: return alert == tmc2209::B_GROUND_SHORT_CLEARED;
        then:
          - logger.log: B_GROUND_SHORT_CLEARED
```



[datasheet]: <./docs/TMC2209_datasheet_rev1.09.pdf> "Datasheet rev 1.09"

[config-id]: <https://esphome.io/guides/configuration-types#config-id> "ESPHome ID Config Schema"
[config-pin]: <https://esphome.io/guides/configuration-types#config-pin-schema> "ESPHome Pin Config Schema"
[config-templatable]: <https://esphome.io/automations/templates#config-templatable> "Templatable configuration"
[uart-component]: <https://esphome.io/components/uart.html> "ESPHome UART Config"
[base-stepper-component]: <https://esphome.io/components/stepper/#base-stepper-configuration> "ESPHome Base Stepper Component"
[base-sensor-component]: <https://esphome.io/components/sensor/#config-sensor> "ESPHome Base Sensor Component"

[highfrequencylooprequester]: <https://github.com/esphome/esphome/blob/9713458368dfb9fd9aab8016cfe8c85d77b04887/esphome/core/helpers.h#L609> "HighFrequencyLoopRequester class"

[tmcapi-tmc2209-hwa]: <https://github.com/slimcdk/TMC-API/blob/master/tmc/ic/TMC2209/TMC2209_HW_Abstraction.h> "TMC-API TMC2209 Hardware Abstractions"

