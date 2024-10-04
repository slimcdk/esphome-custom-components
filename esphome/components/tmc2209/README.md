# TMC2209

ESPHome component to interact with a TMC2209 stepper motor driver over UART and regular step/dir.


<p align="center">
  <img src="./docs/trinamic-bob-module.jpg" alt="Trinamic BOB" width="19%" />
  <img src="./docs/silentstepstick-module.jpg" alt="SilentStepStick" width="19%" />
  <img src="./docs/bigtreetech-module.webp" alt="BigTreeTech" width="19%" />
  <img src="./docs/fysetc-module.webp" alt="Fysetc" width="19%" />
  <img src="./docs/grobo-module.jpg" alt="GRobotronics" width="19%" />
</p>

> [!IMPORTANT]
*Only a single `tmc2209` instance (device) per UART config is currently supported by ESPHome. Multiple drivers require multiple UART connections.*


# Table of contents
- [Config](#config)
  - [UART Setup](#configuration-of-uart-bus)
  - [Stepper Configuration](#stepper-configuration)
    - [Control via UART](#control-the-position-using-serial-uart)
    - [Control via pulses](#control-the-position-using-traditional-stepping-pulses-and-direction)
- [Automation](#automation)
  - [`on_alert` Trigger](#on_alert)
- [Actions](#actions)
  - [`tmc2209.configure` Action](#tmc2209configure-action)
  - [`tmc2209.enable` Action](#tmc2209enable-action)
  - [`tmc2209.disable` Action](#tmc2209disable-action)
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
    components: [ tmc2209, stepper ]
```
---

### Configuration of [UART Bus][uart-component].

```yaml
uart:
  tx_pin: REPLACEME
  rx_pin: REPLACEME
  baud_rate: 512000 # 9600 -> 500k
```

* `baud_rate` (**Required**, int): The baud rate of the UART bus. TMC2209 will auto-detect baud rates from 9600 to 500k with the internal clock/oscillator. An external clock/oscillator is needed for baud rates higher than 500k.

> [!CAUTION]
***A lot is happening over serial and low baud rates might cause warnings about the component taking too long. Use something like 115200 or higher.***

* `tx_pin` (*Optional*, [Output Pin Schema][config-pin]): This is the ESPHome device's transmit pin. This should be connected through a 1k Ohm resistor to `PDN_UART` on the TMC2209.

* `rx_pin` (*Optional*, [Input Pin Schema][config-pin]): This is the ESPHome device's receive pin. This should be connected directly to `PDN_UART` on the TMC2209.

> [!IMPORTANT]
*Avoid selecting a UART which is utilized for other purposes. For instance boot log as the TMC2209 will try to interpret the output.*

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
  - platform: tmc2209
    id: driver
    address: 0x00                 # optional, default is 0x00
    enn_pin: REPLACEME            # optional, mostly not needed
    diag_pin: REPLACEME           # optional, but highly recommended to set
    index_pin: REPLACEME          # optional, unless using uart control
    step_pin: REPLACEME           # optional, unless using step/dir control
    dir_pin: REPLACEME            # optional, unless using step/dir control
    rsense: REPLACEME             # optional, check resistor on board
    vsense: False                 # optional, default is False
    ottrim: 0                     # optional, default is OTP
    clock_frequency: 12MHz        # optional, default is 12MHz
    poll_status_interval: 500ms   # optional, default is 500ms and has no effect if diag_pin is set

    max_speed: 500 steps/s
    acceleration: 2500 steps/s^2  # optional, default is INF (no soft acceleration)
    deceleration: 2500 steps/s^2  # optional, default is INF (no soft deceleration)
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the stepper so that you can control it.

* `address` (*Optional*, hex): UART address of the IC. Configured by setting MS1_AD0 or MS2_AD1 high or low. Default is `0x00`.

* `enn_pin` (*Optional*, [Output Pin Schema][config-pin]): Enable not input pin for the driver.
    > *Driver can't be enabled if ENN is left floating. Either configure it if it's actually connected or wire it to GND.*

* `diag_pin` (*Optional*, [Input Pin Schema][config-pin]): Driver error signaling from the driver.

* `index_pin` (*Optional*, [Input Pin Schema][config-pin]): Serves as stepping feedback from the internal step pulse generator.

* `step_pin` (*Optional*, [Output Pin Schema][config-pin]): Provides stepping pulses to the driver.

* `dir_pin` (*Optional*, [Output Pin Schema][config-pin]): Controls direction of the motor.

* `rsense` (*Optional*, resistance): Motor current sense resistors. Often varies from ~75 to 1000 mOhm. The actual value for your board can be found in the documentation. Leave empty to enable internal sensing using RDSon (170 mOhm). *Don't leave empty if your board has external sense resistors!*

* `vsense` (*Optional*, boolean): Reduce currents/power to ~55% if smaller (<1/4 W) RSense resistors are used. Defaults to OTP. RSense must be configued as well.

* `ottrim` (*Optional*, int): Limits for warning and shutdown temperatures. Default is OTP. OTP is 0 from factory. See below table for values.
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

    > *Driver will stay disabled until prewarning clears when shutdown has been triggered. Can be reenabled once temperature is below prewarning.*

* `clock_frequency` (*Optional*, frequency): Timing reference for all functionalities of the driver. Defaults to 12MHz, which all drivers are factory calibrated to. Only set if using external clock.

* `poll_status_interval` (*Optional*, [Time][config-time]): Interval to poll driver for fault indicators like overtemperature, open load, short etc (***This does not set detection interval for stall***). Default is 500ms. This has no effect if `diag_pin` is configured.

* All other from [Base Stepper Component][base-stepper-component]


## Automation

### `on_alert`
An alert event is fired whenever a driver warning or error is detected. For instance when the motor stalls. This event can be used for sensorless homing. Works both with control over serial (UART) and with stepping and direction pulses.
```yaml
stepper:
  - platform: tmc2209
    id: driver
    ...
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2209::STALLED;
          then:
            - logger.log: "Motor stalled!"
            - stepper.stop: driver
```
> <small>All events in an [example config](#alert-events)  for easy copy/paste</small>

#### Currently supported alert events

Most alerts is signaling that the driver is in a given state. The majority of alert events also has a `CLEARED` or similar counterpart signaling that the driver is now not in the given state anymore.

  * `DIAG_TRIGGERED` DIAG output is triggered. Primarily driver errors.
  * `STALLED` Motor is considered stalled when SG_RESULT crosses SGTHRS. Check below for more.
  * `OVERTEMPERATURE_PREWARNING` | `OVERTEMPERATURE_PREWARNING_CLEARED` Driver is warning about increasing temperature.
  * `OVERTEMPERATURE` | `OVERTEMPERATURE_CLEARED` Driver is at critical high temperature and is shutting down.
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

> [!NOTE]
*`STALLED` is the event you would want for sensorless homing. Check the [sensorless homing example](#sensorless-homing)*.

> [!IMPORTANT]
*Stall monitoring becomes enabled when the motor has a velocity equal to half of the configured max speed. Since this component/setup is expected to be an open loop system, the velocity is estimated from max speed, acceleration and deceleration and not the actual velocity of the motor. This applies to both UART and pulse control.*


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

* `id` (**Required**, ID): Reference to the stepper tmc2209 component. Can be left out if only a single TMC2209 is configured.

* `microsteps` (*Optional*, int, [templatable][config-templatable]): Microstepping. Possible values are `1`, `2`, `4`, `8`, `16`, `32`, `64`, `128`, `256`.

* `inverse_direction` (*Optional*, bool, [templatable][config-templatable]): Inverse the rotational direction.

* `tcool_threshold` (*Optional*, int, [templatable][config-templatable]): Value for the COOLSTEP TCOOL threshold.

* `stallguard_threshold` (*Optional*, int, [templatable][config-templatable]): Value for the StallGuard4 threshold.

* `interpolation` (*Optional*, bool, [templatable][config-templatable]): The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for the smoothest motor operation.

* `irun` (*Optional*, int, [templatable][config-templatable]): IRUN setting. Must be between 0 and 31.

* `ihold` (*Optional*, int, [templatable][config-templatable]): IHOLD setting. Must be between 0 and 31.

* `tpowerdown` (*Optional*, int, [templatable][config-templatable]): TPOWERDOWN setting. Must be between 0 and 31.

* `iholddelay` (*Optional*, int, [templatable][config-templatable]): IHOLDDELAY setting. Must be between 0 and 31.

* `run_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IRUN based on RSense and VSense according to [section 9][datasheet].

* `hold_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IHOLD based on RSense and VSense according to [section 9][datasheet].

* `standstill_mode` (*Optional*, [templatable][config-templatable]): Standstill mode for when movement stops. Default is OTP. Available modes are:
  * `normal`: Actively breaks the motor.
  * `freewheeling`: `ihold` and/or `hold_current` must be 0 for true freewheeling. Higher IHOLD enforces high currents thus harder braking effect.
  * `short_coil_ls`: Similar to `freewheeling`, but with motor coils shorted to low side voltage.
  * `short_coil_hs`: Similar to `freewheeling`, but with motor coils shorted to high side voltage.

* `enable_spreadcycle` (*Optional*, bool, [templatable][config-templatable]): `True` completely disables StealthChop and only uses SpreadCycle, `False` allows use of StealthChop. Defaults to OTP.

* `stall_detection_activation_level` (*Optional*, percentage, [templatable][config-templatable]): Threshold is percentage of *max speed* for when stall detection is active. Default is 50% which should work fine. 0% might cause false triggers on acceleration or deceleration.


> [!NOTE]
*Registers and fields on the driver persist through an ESPHome device reboot, so previously written states may remain active*

*See [section 1.7][datasheet] for visiual graphs of IRUN, TPOWERDOWN and IHOLDDELAY and IHOLD*


### `tmc2209.enable` Action

Enables driver on ENN (requires `enn_pin`) or by setting *TOFF* to 3.

```yaml
on_...:
  - tmc2209.enable: driver
```
* `id` (**Required**, ID): Reference to the stepper tmc2209 component. Can be left out if only a single TMC2209 is configured.
> [!NOTE]
*Driver will automatically be enabled if new target is issued.*


### `tmc2209.disable` Action

Disables driver on ENN (requires `enn_pin`) or by setting *TOFF* to 0.

```yaml
on_...:
  - tmc2209.disable: driver
```
* `id` (**Required**, ID): Reference to the stepper tmc2209 component. Can be left out if only a single TMC2209 is configured.




### Sensors

Some metrics from the driver is exposed as a ready-to-use sensor component.

> [!IMPORTANT]
*Only intended for diagnostic purposes*

```yaml
sensor:
  - platform: tmc2209
    type: stallguard_result
    name: Driver stallguard
    update_interval: 250ms

  - platform: tmc2209
    type: motor_load
    name: Motor load
    update_interval: 250ms

  - platform: tmc2209
    type: actual_current
    name: Actual current
    update_interval: 250ms
```
* `tmc2209_id` (*Optional*, [ID][config-id]): Manually specify the ID of the `stepper.tmc2209` you want to use this sensor.

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
  - platform: tmc2209
    id: driver
    ...
    on_alert:
      - if:
          condition:
            lambda: return alert == tmc2209::STALLED;
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
    components: [ tmc2209, stepper ]

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
  - platform: tmc2209
    type: motor_load
    name: Motor load
    update_interval: 250ms
```

Output of above configuration. Registers could differ due to OTP.
```console
...
[00:00:00][C][tmc2209:422]: TMC2209 Stepper:
[00:00:00][C][tmc2209:425]:   Control: Serial with feedback
[00:00:00][C][tmc2209:431]:   Acceleration: 1500 steps/s^2
[00:00:00][C][tmc2209:431]:   Deceleration: 500 steps/s^2
[00:00:00][C][tmc2209:431]:   Max Speed: 900 steps/s
[00:00:00][C][tmc2209:438]:   Detected IC version: 0x21
[00:00:00][C][tmc2209:443]:   ENN Pin: GPIO38
[00:00:00][C][tmc2209:444]:   DIAG Pin: GPIO41
[00:00:00][C][tmc2209:448]:   INDEX Pin: GPIO42
[00:00:00][C][tmc2209:452]:   Address: 0x00
[00:00:00][C][tmc2209:453]:   Microsteps: 8
[00:00:00][C][tmc2209:459]:   RSense: 0.110 Ohm (external sense resistors)
[00:00:00][C][tmc2209:463]:     Configured for high heat dissipation (vsense = false)
[00:00:00][C][tmc2209:468]:   Overtemperature: prewarning = 120C | shutdown = 143C
[00:00:00][C][tmc2209:469]:   Clock frequency: 12000000 Hz
[00:00:00][C][tmc2209:471]:   Register dump:
[00:00:00][C][tmc2209:472]:     GCONF:        0x000000E0
[00:00:00][C][tmc2209:473]:     GSTAT:        0x00000001
[00:00:00][C][tmc2209:474]:     IFCNT:        0x00000016
[00:00:00][C][tmc2209:475]:     SLAVECONF:    0x00000000
[00:00:00][C][tmc2209:476]:     OTP_PROG:     0x00000000
[00:00:00][C][tmc2209:477]:     OTP_READ:     0x0000000A
[00:00:00][C][tmc2209:478]:     IOIN:         0x21000240
[00:00:00][C][tmc2209:479]:     FACTORY_CONF: 0x0000000A
[00:00:00][C][tmc2209:480]:     IHOLD_IRUN:   0x00000C00
[00:00:00][C][tmc2209:481]:     TPOWERDOWN:   0x00000000
[00:00:00][C][tmc2209:482]:     TSTEP:        0x000FFFFF
[00:00:00][C][tmc2209:483]:     TPWMTHRS:     0x00000000
[00:00:00][C][tmc2209:484]:     TCOOLTHRS:    0x00000000
[00:00:00][C][tmc2209:485]:     VACTUAL:      0x00000000
[00:00:00][C][tmc2209:486]:     SGTHRS:       0x00000032
[00:00:00][C][tmc2209:487]:     SG_RESULT:    0x00000000
[00:00:00][C][tmc2209:488]:     COOLCONF:     0x00000000
[00:00:00][C][tmc2209:489]:     MSCNT:        0x00000250
[00:00:00][C][tmc2209:490]:     MSCURACT:     0x0126018A
[00:00:00][C][tmc2209:491]:     CHOPCONF:     0x15010053
[00:00:00][C][tmc2209:492]:     DRV_STATUS:   0xC0000000
[00:00:00][C][tmc2209:493]:     PWMCONF:      0xC81D0E24
[00:00:00][C][tmc2209:494]:     PWMSCALE:     0x006C006D
[00:00:00][C][tmc2209:495]:     PWM_AUTO:     0x000E0024
...
```



### Advanced

Writing to and reading from registers and register fields from the driver can easily be done with the help of preexisting [helper definitions][tmcapi-tmc2209-hwa] from the underlying TMC-API. A description of the register map can be found under [section 5][datasheet].
> [!IMPORTANT]
***The `tmc2209` component holds a mirror in memory of the values written to the driver. This means write-only registers can still be read with below methods provided they have been written already.***
>
>*Definitions ending in `_MASK` or `_SHIFT` should not be used.*

The `tmc2209` base component exposes four methods:
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

    // Write value 3 to MRES register field. 2**3 = microstepping of 8
  - platform: template
    name: Set microstepping to 8
    on_press:
      - lambda: id(driver)->write_field(TMC2209_MRES_FIELD, 3);

```


## Wiring
Guides to wire ESPHome supported MCU to a TMC2209 driver for either only UART control or pulse control.

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
* https://www.analog.com/en/products/tmc2209.html
* https://www.programming-electronics-diy.xyz/2023/12/tmc2209-stepper-driver-module-tutorial.html



## Troubleshooting

#### Unable to read IC version. Is the driver powered and wired correctly?
1. Make sure UART is correctly wired and the 1k Ohm resistor is placed correctly.
2. Make sure the driver is power on VM / VS (motor supply voltage). Must be between 4.75 and 29V.

#### Detected unknown IC version: 0x??
First generation of TMC2209s have version `0x21`. There is only a single version released as of Q3 2024. If you are seeing version `0x20` that means you have a TMC2208 which is not supported by this component.

#### Reading from UART timed out at byte 0!
Poor signal integrity can cause instability in the UART connection. The component doesn't retry writing/reading if a reading failed. Make sure the connection is reliable for best performance. Try lower baud rates if these only appear occasionally.

#### Driver makes "sizzling" noise
Long wires connected to ENN might pick up interference causing the driver to make a sizzling noise if left floating.



## TODOs
* Reconfigure driver if driver was power cycled.
* OTTRIM not setting or reading properly.
* Generate step pulses outside of main loop.
* Use index for warning if stepper is controlled with step/dir.

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
[config-time]: <https://esphome.io/guides/configuration-types#config-time> "ESPHome Time Config Schema"
[uart-component]: <https://esphome.io/components/uart.html> "ESPHome UART Config"
[base-stepper-component]: <https://esphome.io/components/stepper/#base-stepper-configuration> "ESPHome Base Stepper Component"
[base-sensor-component]: <https://esphome.io/components/sensor/#config-sensor> "ESPHome Base Sensor Component"

[highfrequencylooprequester]: <https://github.com/esphome/esphome/blob/9713458368dfb9fd9aab8016cfe8c85d77b04887/esphome/core/helpers.h#L609> "HighFrequencyLoopRequester class"

[tmcapi-tmc2209-hwa]: <https://github.com/slimcdk/TMC-API/blob/master/tmc/ic/TMC2209/TMC2209_HW_Abstraction.h> "TMC-API TMC2209 Hardware Abstractions"

