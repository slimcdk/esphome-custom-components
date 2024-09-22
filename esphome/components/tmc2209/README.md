# TMC2209

> [!CAUTION]
*This is still active development so expect some things to change.*

ESPHome component to control a stepper motor using an ADI (formerly Trinamic) TMC2209 stepper motor driver over UART. Technical information can be found in the [section 1.2.1][datasheet].

This implementation can control the stepper purely over serial and through stepping pulses.

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
    components: [ tmc2209 ]
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
> [!NOTE]
*TMC2209 will auto-detect baud rates from 9600 to 500k with the internal clock/oscillator. An external clock/oscillator is needed for baud rates higher than 500k.*
---

### Stepper configuration

The stepper can be controlled in two ways.

#### Using serial (UART)
Accuracy is slightly reduced in favor of tight timings and high-frequency stepping pulses. Pulse generation is unaffected by ESPHome's handling of other components or main thread execution. This means that the host microcontroller (e.g., ESP32) running ESPHome doesn't provide the step generation, but it is handled internally by the driver. Highly recommended for use with high microstep interpolation or true silent operation.

Relevant info can be found in [section 1.3][datasheet].

>[!NOTE]
*The provided accuracy is often precise enough, but it depends on speeds and how often the component can write to the driver.*

>[!IMPORTANT]
*Configure `index_pin` and not `step_pin` and `dir_pin` for this method.*


#### Using traditional stepping pulses and direction
Stepping pulses are handled by the main thread but utilize [increased execution frequency functionality][highfrequencylooprequester] to generate pulses as fast as possible. Pulses are therefore limited to whenever the ESP can generate a pulse, and any timing inconsistencies become audible when the motor runs.

>[!NOTE]
*More components take up more resources slowing the main thread.*

>[!IMPORTANT]
*Configure `step_pin` and `dir_pin` and not `index_pin` for this method.*

>[!CAUTION]
*Current timing in step generation is very instable and cuases the driver to provide unreliable stallguard feedback and stepping inconsistencies*


####

```yaml
stepper:
  - platform: tmc2209
    id: driver
    address: 0x00
    enn_pin: REPLACEME
    diag_pin: REPLACEME # highly recommended to set

    # configure this pin if you intend to use the internal pulse generation of the TMC2209
    index_pin: REPLACEME

    # configure these pins if you intend to have ESPHome generate the stepping pulses
    step_pin: REPLACEME
    dir_pin: REPLACEME

    rsense: 110 mOhm
    vsense: False
    ottrim: 0
    clock_frequency: 12MHz

    max_speed: 500 steps/s
    acceleration: 1000 steps/s^2
    deceleration: 1000 steps/s^2
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the stepper so that you can control it.

* `address` (*Optional*, hex): UART address of the IC. Configured by setting MS1_AD0 or MS2_AD1 high or low. Default is `0x00`.

* `enn_pin` (*Optional*, [Output Pin Schema][config-pin]): Enable not input pin for the driver. No need for manual inverted config as inverted logic is handled internally.

* `diag_pin` (*Optional*, [Input Pin Schema][config-pin]): Driver error signaling from the driver.
  >If not defined, the less reliable detection over UART will be used instead.

* `index_pin` (*Optional*, [Input Pin Schema][config-pin]): Serves as stepping feedback from the internal step pulse generator.

* `step_pin` (*Optional*, [Output Pin Schema][config-pin]): Provides stepping pulses to the driver.

* `dir_pin` (*Optional*, [Output Pin Schema][config-pin]): Controls direction of the motor.

* `rsense` (*Optional*, resistance): Motor current sense resistors. Varies from ~75 to 1000 mOhm. Consult [section 8][datasheet] for a lookup table.

* `vsense` (*Optional*, boolean): Driver uses smaller (<1/4 W) RSense resistors.

* `ottrim` (*Optional*, int): Limits for warning and shutdown temperatures. Default is OTP which is factory configuration of 120C for prewarning and 143C for overtemperature (shutdown). See below table for values.
  <table>
    <thead>
      <tr><th>OTTRIM</th><th>Prewarning</th><th>Shutdown</th></tr>
    </thead>
    <tbody>
      <tr><td>0</td><td>120C</td><td>143C</td></tr>
      <tr><td>1</td><td>120C</td><td>150C</td></tr>
      <tr><td>2</td><td>143C</td><td>150C</td></tr>
      <tr><td>3</td><td>143C</td><td>157C</td></tr>
    </tbody>
  </table>

  >[!NOTE] Driver will stay disabled until prewarning clears when shutdown has been triggered. Can be reenabled once temperature is below prewarning and `ENN` has been toggled.

  >[!WARNING]
  *Configuring `overtemperature` has no effect at the moment as there is an issue with updating the register field on the driver. It will however still monitor temperatures and emit alert events and the driver will also shutdown when temperature is too high*

* `clock_frequency` (*Optional*, frequency): Timing reference for all functionalities of the driver. Defaults to 12MHz, which all drivers are factory calibrated to. Only set if using external clock.

* All other from [Base Stepper Component][base-stepper-component]



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
  * `CP_UNDERVOLTAGE` | `CP_UNDERVOLTAGE_CLEARED` Undervoltage on chargepump input.

>[!NOTE]
*`STALLED` is the event you would want for sensorless homing*

## Actions

### `tmc2209.configure` Action
Example of configuring the driver. For instance on boot.
```yaml
esphome:
  ...
  on_boot:
    - tmc2209.configure:
        microsteps: 8
        stallguard_threshold: 50
        standstill_mode: freewheeling
        run_current: 800mA
        hold_current: 0mA
        tpowerdown: 0
        iholddelay: 0
```

* `id` (**Required**, ID): Reference to the stepper tmc2209 (base, not stepper) component.

* `microsteps` (*Optional*, int, [templatable][config-templatable]): Microstepping. Possible values are `1`, `2`, `4`, `8`, `16`, `32`, `64`, `128`, `256`.

* `inverse_direction` (*Optional*, bool, [templatable][config-templatable]): Inverse the rotational direction.

* `tcool_threshold` (*Optional*, int, [templatable][config-templatable]): Value for the COOLSTEP TCOOL threshold.

* `stallguard_threshold` (*Optional*, int, [templatable][config-templatable]): Value for the StallGuard2 threshold.

* `interpolation` (*Optional*, bool, [templatable][config-templatable]): The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for the smoothest motor operation.

* `irun` (*Optional*, int, [templatable][config-templatable]): IRUN setting. Must be between 0 and 31.

* `ihold` (*Optional*, int, [templatable][config-templatable]): IHOLD setting. Must be between 0 and 31.

* `tpowerdown` (*Optional*, int, [templatable][config-templatable]): TPOWERDOWN setting. Must be between 0 and 31.

* `iholddelay` (*Optional*, int, [templatable][config-templatable]): IHOLDDELAY setting. Must be between 0 and 31.

* `run_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IRUN based on RSense according to [section 9][datasheet].

* `hold_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IHOLD based on RSense according to [section 9][datasheet].

* `standstill_mode` (*Optional*, [templatable][config-templatable]): Standstill mode for when movement stops. Default is OTP. Available modes are:
  * `normal`: Actively breaks the motor
  * `freewheeling`: `ihold` or/and `hold_current` must be 0 for true freewheeling. Otherwise it will act as breaking.
  * `short_coil_ls`:
  * `short_coil_hs`:

>[!NOTE]
*See [section 1.7][datasheet] for *

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
* `tmc2209_id` (*Optional*, [ID][config-id]): Manually specify the ID of the `stepper.tmc2209` you want to use this sensor.

* `type` (**Required**):
  * `stallguard_result` Stator angle shift detected by the driver.
  * `motor_load` Percentage off stall calculated from StallGuard result and set StallGuard threshold. 100% = stalled
  * `actual_current` Active current setting. Either IRUN or IHOLD value.

* All other from [Sensor][base-sensor-component]


## Example config
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [tmc2209]

# esp32 or esp8266 config..

wifi:
  ssid: !secret WIFI_SSID
  password: !secret WIFI_PASSWORD

esphome:
  name: actuator
  on_boot:
    - tmc2209.configure:
        microsteps: 8
        interpolation: true
        tcool_threshold: 400
        stallguard_threshold: 50
        standstill_mode: freewheeling
        run_current: 800mA
        hold_current: 0mA
        tpowerdown: 0
        iholddelay: 0

uart:
  tx_pin: 16
  rx_pin: 17
  baud_rate: 512000


stepper:
  - platform: tmc2209
    id: driver
    max_speed: 500 steps/s
    acceleration: 1000 steps/s^2
    deceleration: 1000 steps/s^2
    rsense: 110 mOhm
    vsense: True
    ottrim: 0
    diag_pin: 41
    index_pin: 42
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2209::STALLED;
          then:
            - logger.log: "Motor stalled!"
            - stepper.stop: driver

button:
  - platform: template
    name: Stop
    on_press:
      - stepper.stop: driver

  - platform: template
    name: 1000 Steps forward
    on_press:
      - stepper.set_target:
          id: driver
          target: !lambda return id(driver).current_position +1000;

  - platform: template
    name: 1000 Steps backward
    on_press:
      - stepper.set_target:
          id: driver
          target: !lambda return id(driver).current_position -1000;

number:
  - platform: template
    name: Target position
    min_value: -100000
    max_value: 100000
    step: 100
    lambda: return id(driver)->current_position;
    update_interval: 1s
    set_action:
      - stepper.set_target:
          id: driver
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
[00:00:00][C][tmc2209:204]: TMC2209 Stepper:
[00:00:00][C][tmc2209:207]:   Control: UART with feedback
[00:00:00][C][tmc2209:215]:   DIAG Pin: GPIO41
[00:00:00][C][tmc2209:216]:   INDEX Pin: GPIO42
[00:00:00][C][tmc2209:219]:   Address: 0x00
[00:00:00][C][tmc2209:226]:   Detected IC version: 0x21
[00:00:00][C][tmc2209:232]:   Overtemperature: prewarning = 120C | shutdown = 143C
[00:00:00][C][tmc2209:233]:   Clock frequency: 12000000 Hz
[00:00:00][C][tmc2209:235]:   Register dump:
[00:00:00][C][tmc2209:236]:     GCONF: 0xE0
[00:00:00][C][tmc2209:237]:     GSTAT: 0x1
[00:00:00][C][tmc2209:238]:     IFCNT: 0xF2
[00:00:00][C][tmc2209:239]:     SLAVECONF: 0x0
[00:00:00][C][tmc2209:240]:     OTP_PROG: 0x0
[00:00:00][C][tmc2209:241]:     OTP_READ: 0xA
[00:00:00][C][tmc2209:242]:     IOIN: 0x21000240
[00:00:00][C][tmc2209:243]:     FACTORY_CONF: 0xA
[00:00:00][C][tmc2209:244]:     IHOLD_IRUN: 0x0
[00:00:00][C][tmc2209:245]:     TPOWERDOWN: 0x0
[00:00:00][C][tmc2209:246]:     TSTEP: 0xFFFFF
[00:00:00][C][tmc2209:247]:     TPWMTHRS: 0x0
[00:00:00][C][tmc2209:248]:     TCOOLTHRS: 0x190
[00:00:00][C][tmc2209:249]:     VACTUAL: 0x0
[00:00:00][C][tmc2209:250]:     SGTHRS: 0x32
[00:00:00][C][tmc2209:251]:     SG_RESULT: 0x0
[00:00:00][C][tmc2209:252]:     COOLCONF: 0x0
[00:00:00][C][tmc2209:253]:     MSCNT: 0x10
[00:00:00][C][tmc2209:254]:     MSCURACT: 0xF60018
[00:00:00][C][tmc2209:255]:     CHOPCONF: 0x15030053
[00:00:00][C][tmc2209:256]:     DRV_STATUS: 0xC0000000
[00:00:00][C][tmc2209:257]:     PWMCONF: 0xC81D0E24
[00:00:00][C][tmc2209:258]:     PWMSCALE: 0x180019
[00:00:00][C][tmc2209:259]:     PWM_AUTO: 0xE0024
[00:00:00][C][tmc2209:261]:   Acceleration: 1000 steps/s^2
[00:00:00][C][tmc2209:261]:   Deceleration: 1000 steps/s^2
[00:00:00][C][tmc2209:261]:   Max Speed: 500 steps/s
...
```



### Advanced

Writing to and reading from registers and register fields from the driver can easily be done with the help of preexisting [helper definitions][tmcapi-tmc2209-hwa] from the underlying TMC-API. A description of the register map can be found under [section 5][datasheet].
> [!IMPORTANT]
**The `tmc2209` component holds a mirror in memory of the values written to the driver. This means write-only registers can still be read with below methods provided they have been written already.**
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
* Reconfigure driver if driver was power cycled.
* Driver warning and error trigger events.
* OTTRIM not setting or reading properly
* Generate step pulses outside of main loop


## MISC

### Alert events
All alert events configured for easy copy-pasta.
```yaml
stepper:
  - platform: tmc2209
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
      - if:
          condition:
            lambda: return alert == tmc2209::CP_UNDERVOLTAGE;
          then:
            - logger.log: CP_UNDERVOLTAGE
      - if:
          condition:
            lambda: return alert == tmc2209::CP_UNDERVOLTAGE_CLEARED;
          then:
            - logger.log: CP_UNDERVOLTAGE_CLEARED
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

