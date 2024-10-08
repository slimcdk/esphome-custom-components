# TMC2300

ESPHome component to interact with a TMC2300 stepper motor driver over UART and regular step/dir.


<p align="center">
  <img src="./docs/trinamic-bob-module.jpg" alt="Trinamic BOB" width="19%" />
  <img src="./docs/silentstepstick-module.jpg" alt="SilentStepStick" width="19%" />
  <img src="./docs/bigtreetech-module.webp" alt="BigTreeTech" width="19%" />
  <img src="./docs/fysetc-module.webp" alt="Fysetc" width="19%" />
  <img src="./docs/grobo-module.jpg" alt="GRobotronics" width="19%" />
</p>

> [!IMPORTANT]
*Only a single `tmc2300` instance (device) per UART config is currently supported by ESPHome. Multiple drivers require multiple UART connections.*


# Table of contents
- [Config](#config)
  - [UART Setup](#configuration-of-uart-bus)
  - [Stepper Configuration](#stepper-configuration)
    - [Control via UART](#control-the-position-using-serial-uart)
    - [Control via pulses](#control-the-position-using-traditional-stepping-pulses-and-direction)
- [Automation](#automation)
  - [`on_alert` Trigger](#on_alert)
- [Actions](#actions)
  - [`tmc2300.configure` Action](#tmc2300configure-action)
  - [`tmc2300.enable` Action](#tmc2300enable-action)
  - [`tmc2300.disable` Action](#tmc2300disable-action)
- [Driver Sensors](#sensors)
- [Examples](#example-config)
- [Advanced](#advanced)
- [Wiring](#wiring)
  - [UART Control Wiring](#uart-control)
  - [Pulse Control Wiring](#pulse-control)
- [Resources](#resources)
- [Troubleshooting](#troubleshooting)



## Config

Import the component(s).
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ tmc2300, stepper ]
```
---

### Configuration of [UART Bus][uart-component].

```yaml
uart:
  tx_pin: REPLACEME
  rx_pin: REPLACEME
  baud_rate: 512000 # 9600 -> 500k
```

* `baud_rate` (**Required**, int): The baud rate of the UART bus. TMC2300 will auto-detect baud rates from 9600 to 500k with the internal clock/oscillator. An external clock/oscillator is needed for baud rates higher than 500k.

> [!CAUTION]
***A lot is happening over serial and low baud rates might cause warnings about the component taking too long. Use something like 115200 or higher.***

* `tx_pin` (*Optional*, [Output Pin Schema][config-pin]): This is the ESPHome device's transmit pin. This should be connected through a 1k Ohm resistor to `PDN_UART` on the TMC2300.

* `rx_pin` (*Optional*, [Input Pin Schema][config-pin]): This is the ESPHome device's receive pin. This should be connected directly to `PDN_UART` on the TMC2300.

> [!IMPORTANT]
*Avoid selecting a UART which is utilized for other purposes. For instance boot log as the TMC2300 will try to interpret the output.*

---

### Stepper configuration

> *The stepper can be controlled in two ways. See [section 1.3][datasheet] for technical information.*


#### Control the position using serial (UART)

Accuracy is slightly reduced for tighter timing and high-frequency pulse generation, which is handled internally by the driver rather than the ESPHome microcontroller. This ensures consistent pulse generation without interference from other components, making it ideal for high microstep interpolation or silent operation. Relevant info can be found in [section 1.3][datasheet].

> *The provided accuracy is often precise enough, but it depends on speeds and how often the component can write to the driver.*

> [!IMPORTANT]
*Configure `index_pin` and not `step_pin` and `dir_pin` for this method.*



#### Control the position using traditional stepping pulses and direction

Stepping pulses are handled by the main thread but utilize [increased execution frequency functionality][highfrequencylooprequester] to generate pulses as fast as possible. Pulses are therefore limited to whenever the ESP can generate a pulse, and any timing inconsistencies may cause erratic motor noise or operational issues when running the motor.

> *More components take up more resources slowing the main thread.*

> [!IMPORTANT]
*Configure `step_pin` and `dir_pin` and not `index_pin` for this method. Stay below 8 microstepping for best performance.*


####

```yaml
stepper:
  - platform: tmc2300
    id: driver
    address: 0x00                 # optional, default is 0x00
    enable_pin: REPLACEME         # optional, mostly not needed
    nstdby_pin: REPLACEME         # optional, mostly not needed
    diag_pin: REPLACEME           # optional, unless using uart control
    step_pin: REPLACEME           # optional, unless using step/dir control
    dir_pin: REPLACEME            # optional, unless using step/dir control
    rsense: REPLACEME             # optional, check resistor on board
    clock_frequency: 12MHz        # optional, default is 12MHz
    poll_status_interval: 500ms   # optional, default is 500ms and has no effect if diag_pin is set

    max_speed: 500 steps/s
    acceleration: 2500 steps/s^2  # optional, default is INF (no soft acceleration)
    deceleration: 2500 steps/s^2  # optional, default is INF (no soft deceleration)
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the stepper so that you can control it.

* `address` (*Optional*, hex): UART address of the IC. Configured by setting MS1_AD0 or MS2_AD1 high or low. Default is `0x00`.

* `enable_pin` (*Optional*, [Output Pin Schema][config-pin]): Enable input pin for the driver.

* `nstdby_pin` (*Optional*, [Output Pin Schema][config-pin]): Standby input pin for the driver.

* `diag_pin` (*Optional*, [Input Pin Schema][config-pin]): Serves as stepping feedback from the internal step pulse generator.

* `step_pin` (*Optional*, [Output Pin Schema][config-pin]): Provides stepping pulses to the driver.

* `dir_pin` (*Optional*, [Output Pin Schema][config-pin]): Controls direction of the motor.

* `rsense` (*Optional*, resistance): Motor current sense resistors. Often varies from ~75 to 1000 mOhm. The actual value for your board can be found in the documentation. Leave empty to enable internal sensing using RDSon (170 mOhm). *Don't leave empty if your board has external sense resistors!*

* `clock_frequency` (*Optional*, frequency): Timing reference for all functionalities of the driver. Defaults to 12MHz, which all drivers are factory calibrated to. Only set if using external clock.

* `poll_status_interval` (*Optional*, [Time][config-time]): Interval to poll driver for fault indicators like overtemperature, open load, short etc (***This does not set detection interval for stall***). Default is 500ms. This has no effect if `diag_pin` is configured.

* All other from [Base Stepper Component][base-stepper-component]


## Automation

### `on_alert`
An alert event is fired whenever a driver warning or error is detected. For instance when the motor stalls. This event can be used for sensorless homing. Works both with control over serial (UART) and with stepping and direction pulses.
```yaml
stepper:
  - platform: tmc2300
    id: driver
    ...
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2300::STALLED;
          then:
            - logger.log: "Motor stalled!"
            - stepper.stop: driver
```
> <small>All events in an [example config](#alert-events)  for easy copy/paste</small>

#### Currently supported alert events

Most alerts is signaling that the driver is in a given state. The majority of alert events also has a `CLEARED` or similar counterpart signaling that the driver is now not in the given state anymore.

  * `STALLED` Motor is considered stalled when SG_RESULT crosses SGTHRS. Check below for more.

> [!NOTE]
*`STALLED` is the event you would want for sensorless homing. Check the [sensorless homing example](#sensorless-homing)*.

> [!IMPORTANT]
*Stall monitoring becomes enabled when the motor has a velocity equal to the configured max speed. Since this component/setup is expected to be an open loop system, the velocity is estimated from max speed, acceleration and deceleration and not the actual velocity of the motor. This applies to both UART and pulse control.*


## Actions

### `tmc2300.configure` Action
Example of configuring the driver. For instance on boot.
```yaml
esphome:
  ...
  on_boot:
    - tmc2300.configure:
        microsteps: 8
        stallguard_threshold: 50
        standstill_mode: freewheeling
        run_current: 800mA
        hold_current: 0mA
        tpowerdown: 0
        iholddelay: 0
```

* `id` (**Required**, ID): Reference to the stepper tmc2300 component. Can be left out if only a single TMC2300 is configured.

* `microsteps` (*Optional*, int, [templatable][config-templatable]): Microstepping. Possible values are `1`, `2`, `4`, `8`, `16`, `32`, `64`, `128`, `256`.

* `inverse_direction` (*Optional*, bool, [templatable][config-templatable]): Inverse the rotational direction.

* `tcool_threshold` (*Optional*, int, [templatable][config-templatable]): Value for the COOLSTEP TCOOL threshold.

* `stallguard_threshold` (*Optional*, int, [templatable][config-templatable]): Value for the StallGuard4 threshold.

* `interpolation` (*Optional*, bool, [templatable][config-templatable]): The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for the smoothest motor operation.

* `irun` (*Optional*, int, [templatable][config-templatable]): IRUN setting. Must be between 0 and 31.

* `ihold` (*Optional*, int, [templatable][config-templatable]): IHOLD setting. Must be between 0 and 31.

* `tpowerdown` (*Optional*, int, [templatable][config-templatable]): TPOWERDOWN setting. Must be between 0 and 31.

* `iholddelay` (*Optional*, int, [templatable][config-templatable]): IHOLDDELAY setting. Must be between 0 and 31.

* `run_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IRUN based on RSense according to [section 9][datasheet].

* `hold_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IHOLD based on RSense according to [section 9][datasheet].

* `standstill_mode` (*Optional*, [templatable][config-templatable]): Standstill mode for when movement stops. Default is OTP. Available modes are:
  * `normal`: Actively breaks the motor.
  * `freewheeling`: `ihold` and/or `hold_current` must be 0 for true freewheeling. Higher IHOLD enforces high currents thus harder braking effect.
  * `short_coil_ls`: Similar to `freewheeling`, but with motor coils shorted to low side voltage.
  * `short_coil_hs`: Similar to `freewheeling`, but with motor coils shorted to high side voltage.

* `enable_spreadcycle` (*Optional*, bool, [templatable][config-templatable]): `True` completely disables StealthChop and only uses SpreadCycle, `False` allows use of StealthChop. Defaults to OTP.


> [!NOTE]
*Registers and fields on the driver persist through an ESPHome device reboot, so previously written states may remain active*

*See [section 1.7][datasheet] for visiual graphs of IRUN, TPOWERDOWN and IHOLDDELAY and IHOLD*


### `tmc2300.enable` Action

Enables driver on ENN.

```yaml
on_...:
  - tmc2300.enable: driver
```
* `id` (**Required**, ID): Reference to the stepper tmc2300 component. Can be left out if only a single TMC2300 is configured.

> [!IMPORTANT]
*This action has no effect if `enn_pin` isn't set.*



### `tmc2300.disable` Action

Disables driver on ENN. *Stop* is also called.

```yaml
on_...:
  - tmc2300.disable: driver
```
* `id` (**Required**, ID): Reference to the stepper tmc2300 component. Can be left out if only a single TMC2300 is configured.

> [!IMPORTANT]
*This action has no effect on ENN if `enn_pin` isn't set.* *Driver will be automatically be enabled if new target is issued.*



### Sensors

Some metrics from the driver is exposed as a ready-to-use sensor component.

> [!IMPORTANT]
*Only intended for diagnostic purposes*

```yaml
sensor:
  - platform: tmc2300
    type: stallguard_result
    name: Driver stallguard
    update_interval: 250ms

  - platform: tmc2300
    type: motor_load
    name: Motor load
    update_interval: 250ms

  - platform: tmc2300
    type: actual_current
    name: Actual current
    update_interval: 250ms
```
* `tmc2300_id` (*Optional*, [ID][config-id]): Manually specify the ID of the `stepper.tmc2300` you want to use this sensor.

* `type` (**Required**):
  * `stallguard_result` Stator angle shift detected by the driver.
  * `motor_load` Percentage off stall calculated from StallGuard result and set StallGuard threshold. 100% = stalled
  * `actual_current` Active current setting. Either IRUN or IHOLD value.

* All other from [Sensor][base-sensor-component]



### Sensorless homing

Example config with logic of "homing" the motor against a mechanical hard-stop.

```yaml
esphome:
  ...
  on_boot:
    - button.press: home # home right after boot

globals:
  - id: has_homed
    type: bool
    initial_value: "true"
    restore_value: no

stepper:
  - platform: tmc2300
    id: driver
    ...
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2300::STALLED;
          then:
            - logger.log: "Motor stalled!"
            - stepper.stop: driver
            - if:
                condition:
                  lambda: return !id(has_homed);
                then:
                  - stepper.report_position:
                      id: driver
                      position: 0
                  - globals.set:
                      id: has_homed
                      value: "true"
                  - logger.log: "Home position set"

button:
  - platform: template
    name: Home
    on_press:
      - logger.log: "Going home!"
      - globals.set:
          id: has_homed
          value: "false"
      - stepper.set_target:
          id: driver
          target: -9999999
```



## Example config
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ tmc2300, stepper ]

# esp32 or esp8266 config..

wifi:
  ssid: !secret WIFI_SSID
  password: !secret WIFI_PASSWORD

esphome:
  name: actuator
  on_boot:
    - tmc2300.configure:
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
  - platform: tmc2300
    id: driver
    max_speed: 800 steps/s
    acceleration: 7500 steps/s^2
    deceleration: 7500 steps/s^2
    rsense: 110 mOhm
    vsense: False
    ottrim: 0
    index_pin: 42
    diag_pin: 41
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2300::STALLED;
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
          target: !lambda return id(driver)->current_position +1000;

  - platform: template
    name: 1000 Steps backward
    on_press:
      - stepper.set_target:
          id: driver
          target: !lambda return id(driver)->current_position -1000;

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
  - platform: tmc2300
    type: motor_load
    name: Motor load
    update_interval: 250ms
```

Output of above configuration. Registers could differ due to OTP.
```console
...
[00:00:00][C][tmc2300:421]: TMC2300 Stepper:
[00:00:00][C][tmc2300:424]:   Control: Serial with feedback
[00:00:00][C][tmc2300:430]:   Acceleration: 500 steps/s^2
[00:00:00][C][tmc2300:430]:   Deceleration: 500 steps/s^2
[00:00:00][C][tmc2300:430]:   Max Speed: 800 steps/s
[00:00:00][C][tmc2300:432]:   ENN Pin: GPIO38
[00:00:00][C][tmc2300:434]:   INDEX Pin: GPIO42
[00:00:00][C][tmc2300:437]:   Address: 0x00
[00:00:00][C][tmc2300:438]:   Microsteps: 2
[00:00:00][C][tmc2300:445]:   Detected IC version: 0x21
[00:00:00][C][tmc2300:451]:   Overtemperature: prewarning = 120C | shutdown = 143C
[00:00:00][C][tmc2300:452]:   Clock frequency: 12000000 Hz
[00:00:00][C][tmc2300:453]:   Driver status poll interval: 1000ms
[00:00:00][C][tmc2300:455]:   Register dump:
[00:00:00][C][tmc2300:456]:     GCONF: 0xE0
[00:00:00][C][tmc2300:457]:     GSTAT: 0x1
[00:00:00][C][tmc2300:458]:     IFCNT: 0x18
[00:00:00][C][tmc2300:459]:     SLAVECONF: 0x0
[00:00:00][C][tmc2300:460]:     OTP_PROG: 0x0
[00:00:00][C][tmc2300:461]:     OTP_READ: 0xA
[00:00:00][C][tmc2300:462]:     IOIN: 0x21000240
[00:00:00][C][tmc2300:463]:     FACTORY_CONF: 0xA
[00:00:00][C][tmc2300:464]:     IHOLD_IRUN: 0x1000
[00:00:00][C][tmc2300:465]:     TPOWERDOWN: 0x0
[00:00:00][C][tmc2300:466]:     TSTEP: 0xFFFFF
[00:00:00][C][tmc2300:467]:     TPWMTHRS: 0x0
[00:00:00][C][tmc2300:468]:     TCOOLTHRS: 0x0
[00:00:00][C][tmc2300:469]:     VACTUAL: 0x0
[00:00:00][C][tmc2300:470]:     SGTHRS: 0x32
[00:00:00][C][tmc2300:471]:     SG_RESULT: 0x0
[00:00:00][C][tmc2300:472]:     COOLCONF: 0x0
[00:00:00][C][tmc2300:473]:     MSCNT: 0x40
[00:00:00][C][tmc2300:474]:     MSCURACT: 0xE4005F
[00:00:00][C][tmc2300:475]:     CHOPCONF: 0x17010053
[00:00:00][C][tmc2300:476]:     DRV_STATUS: 0xC0000000
[00:00:00][C][tmc2300:477]:     PWMCONF: 0xC80D0E24
[00:00:00][C][tmc2300:478]:     PWMSCALE: 0x180019
[00:00:00][C][tmc2300:479]:     PWM_AUTO: 0xE002F
...
```



### Advanced

Writing to and reading from registers and register fields from the driver can easily be done with the help of preexisting [helper definitions][tmcapi-tmc2300-hwa] from the underlying TMC-API. A description of the register map can be found under [section 5][datasheet].
> [!IMPORTANT]
***The `tmc2300` component holds a mirror in memory of the values written to the driver. This means write-only registers can still be read with below methods provided they have been written already.***
>
>*Definitions ending in `_MASK` or `_SHIFT` should not be used.*

The `tmc2300` base component exposes four methods:
* `void write_register(uint8_t address, int32_t value)` write/overwrite an entire register.
* `int32_t read_register(uint8_t address)` read an entire registers.
* `void write_field(RegisterField field, uint32_t value)` write/overwrite a register field.
* `uint32_t read_field(RegisterField field)` read a register field.

> [!CAUTION]
*Overwriting some registers may cause instability in the ESPHome component.*


Example usage in lambdas
```yaml
sensor:

    // Read stallguard result (register) into a sensor
  - platform: template
    name: Stallguard result
    lambda: return id(driver)->read_register(TMC2300_SG_RESULT);

    // Read microstep selection index into a sensor. This is a binary exponent like 0,1,2,3,... and microsteps can be calculated like 2**<exponent>
  - platform: template
    name: Microstep selection index
    lambda: return id(driver)->read_field(TMC2300_MRES_FIELD);


number:

    // Write value to stallguard threshold register
  - platform: template
    name: Stallguard threshold
    update_interval: 1s
    min_value: 0
    max_value: 255
    step: 5
    lambda: return id(driver)->read_register(TMC2300_SGTHRS);
    set_action:
      - lambda: id(driver)->write_register(TMC2300_SGTHRS, x);


button:

    // Write value 3 to MRES register field. 2**3 = microstepping of 8
  - platform: template
    name: Set microstepping to 8
    on_press:
      - lambda: id(driver)->write_field(TMC2300_MRES_FIELD, 3);

```


## Wiring
Guides to wire ESPHome supported MCU to a TMC2300 driver for either only UART control or pulse control.

### UART Control
Wiring for [UART control](#control-the-position-using-traditional-stepping-pulses-and-direction)

<img src="./docs/uart-wiring.svg" alt="UART wiring" style="border: 10px solid white" width="100%" />


### Pulse control
Wiring for [Pulse control](#control-the-position-using-serial-uart).

<img src="./docs/sd-wiring.svg" alt="STEP/DIR wiring" style="border: 10px solid white" width="100%" />


> [!IMPORTANT]
*Most drivers come as breakout modules and connections can often be labeled slightly differently. `PDN_UART` was often not labeled, as serial communication was rarly used in the early days, but is apparent on nearly all new modules.*


## Resources

### Parameterization of spreadCycle™
Article by [Bernhard Dwersteg](https://www.analog.com/en/resources/app-notes/an-001.html#author) \
https://www.analog.com/en/resources/app-notes/an-001.html

### Parameterization of StallGuard2™ & CoolStep™
Article by [Bernhard Dwersteg](https://www.analog.com/en/resources/app-notes/an-002.html#author) \
https://www.analog.com/en/resources/app-notes/an-002.html

### Choosing stepper motors
https://docs.duet3d.com/User_manual/Connecting_hardware/Motors_choosing

### Other
* https://www.analog.com/en/products/tmc2300.html
* https://www.programming-electronics-diy.xyz/2023/12/tmc2300-stepper-driver-module-tutorial.html



## Troubleshooting

#### Unable to read IC version. Is the driver powered and wired correctly?
1. Make sure UART is correctly wired and the 1k Ohm resistor is placed correctly.
2. Make sure the driver is power on VM / VS (motor supply voltage). Must be between 4.75 and 29V.

#### Detected unknown IC version: 0x??
First generation of TMC2300s have version `0x21`. There is only a single version released as of Q3 2024. If you are seeing version `0x20` that means you have a TMC2208 which is not supported by this component.

#### Reading from UART timed out at byte 0!
Poor signal integrity can cause instability in the UART connection. The component doesn't retry writing/reading if a reading failed. Make sure the connection is reliable for best performance. Try lower baud rates if these only appear occasionally.

#### Driver makes "sizzling" noise
Long wires connected to ENN might pick up interference causing the driver to make a sizzling noise if left floating.



## TODOs
* Reconfigure driver if driver was power cycled.
* OTTRIM not setting or reading properly
* Generate step pulses outside of main loop


## MISC

### Alert events
All alert events configured for easy copy-pasta.
```yaml
stepper:
  - platform: tmc2300
    id: driver
    ...
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2300::DIAG_TRIGGERED;
          then:
            - logger.log: DIAG_TRIGGERED
      - if:
          condition:
            lambda: return alert == tmc2300::STALLED;
          then:
            - logger.log: STALLED
            - stepper.stop: motor
      - if:
          condition:
            lambda: return alert == tmc2300::OVERTEMPERATURE_PREWARNING;
          then:
            - logger.log: OVERTEMPERATURE_PREWARNING
      - if:
          condition:
            lambda: return alert == tmc2300::OVERTEMPERATURE_PREWARNING_CLEARED;
          then:
            - logger.log: OVERTEMPERATURE_PREWARNING_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::OVERTEMPERATURE;
          then:
            - logger.log: OVERTEMPERATURE_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_ABOVE_120C;
          then:
            - logger.log: TEMPERATURE_ABOVE_120C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_BELOW_120C;
          then:
            - logger.log: TEMPERATURE_BELOW_120C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_ABOVE_143C;
          then:
            - logger.log: TEMPERATURE_ABOVE_143C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_BELOW_143C;
          then:
            - logger.log: TEMPERATURE_BELOW_143C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_ABOVE_150C;
          then:
            - logger.log: TEMPERATURE_ABOVE_150C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_BELOW_150C;
          then:
            - logger.log: TEMPERATURE_BELOW_150C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_ABOVE_157C;
          then:
            - logger.log: TEMPERATURE_ABOVE_157C
      - if:
          condition:
            lambda: return alert == tmc2300::TEMPERATURE_BELOW_157C;
          then:
            - logger.log: TEMPERATURE_BELOW_157C
      - if:
          condition:
            lambda: return alert == tmc2300::A_OPEN_LOAD;
          then:
            - logger.log: A_OPEN_LOAD
      - if:
          condition:
            lambda: return alert == tmc2300::A_OPEN_LOAD_CLEARED;
          then:
            - logger.log: A_OPEN_LOAD_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::B_OPEN_LOAD;
          then:
            - logger.log: B_OPEN_LOAD
      - if:
          condition:
            lambda: return alert == tmc2300::B_OPEN_LOAD_CLEARED;
          then:
            - logger.log: B_OPEN_LOAD_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::A_OPEN_LOAD;
          then:
            - logger.log: A_OPEN_LOAD
      - if:
          condition:
            lambda: return alert == tmc2300::A_OPEN_LOAD_CLEARED;
          then:
            - logger.log: A_OPEN_LOAD_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::B_OPEN_LOAD;
          then:
            - logger.log: B_OPEN_LOAD
      - if:
          condition:
            lambda: return alert == tmc2300::B_OPEN_LOAD_CLEARED;
          then:
            - logger.log: B_OPEN_LOAD_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::A_LOW_SIDE_SHORT;
          then:
            - logger.log: A_LOW_SIDE_SHORT
      - if:
          condition:
            lambda: return alert == tmc2300::A_LOW_SIDE_SHORT_CLEARED;
          then:
            - logger.log: A_LOW_SIDE_SHORT_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::B_LOW_SIDE_SHORT;
          then:
            - logger.log: B_LOW_SIDE_SHORT
      - if:
          condition:
            lambda: return alert == tmc2300::B_LOW_SIDE_SHORT_CLEARED;
          then:
            - logger.log: B_LOW_SIDE_SHORT_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::A_GROUND_SHORT_CLEARED;
          then:
            - logger.log: A_GROUND_SHORT_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::B_GROUND_SHORT;
          then:
            - logger.log: B_GROUND_SHORT
      - if:
          condition:
            lambda: return alert == tmc2300::B_GROUND_SHORT_CLEARED;
          then:
            - logger.log: B_GROUND_SHORT_CLEARED
      - if:
          condition:
            lambda: return alert == tmc2300::CP_UNDERVOLTAGE;
          then:
            - logger.log: CP_UNDERVOLTAGE
      - if:
          condition:
            lambda: return alert == tmc2300::CP_UNDERVOLTAGE_CLEARED;
          then:
            - logger.log: CP_UNDERVOLTAGE_CLEARED
```



[datasheet]: <./docs/TMC2300_datasheet_rev1.09.pdf> "Datasheet rev 1.09"

[config-id]: <https://esphome.io/guides/configuration-types#config-id> "ESPHome ID Config Schema"
[config-pin]: <https://esphome.io/guides/configuration-types#config-pin-schema> "ESPHome Pin Config Schema"
[config-templatable]: <https://esphome.io/automations/templates#config-templatable> "Templatable configuration"
[config-time]: <https://esphome.io/guides/configuration-types#config-time> "ESPHome Time Config Schema"
[uart-component]: <https://esphome.io/components/uart.html> "ESPHome UART Config"
[base-stepper-component]: <https://esphome.io/components/stepper/#base-stepper-configuration> "ESPHome Base Stepper Component"
[base-sensor-component]: <https://esphome.io/components/sensor/#config-sensor> "ESPHome Base Sensor Component"

[highfrequencylooprequester]: <https://github.com/esphome/esphome/blob/9713458368dfb9fd9aab8016cfe8c85d77b04887/esphome/core/helpers.h#L609> "HighFrequencyLoopRequester class"

[tmcapi-tmc2300-hwa]: <https://github.com/slimcdk/TMC-API/blob/master/tmc/ic/TMC2300/TMC2300_HW_Abstraction.h> "TMC-API TMC2300 Hardware Abstractions"

