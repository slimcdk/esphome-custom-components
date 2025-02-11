<a href="https://www.buymeacoffee.com/slimcdk"><img src="https://img.buymeacoffee.com/button-api/?text=Buy me a pizza&emoji=🍕&slug=slimcdk&button_colour=FFDD00&font_colour=000000&font_family=Cookie&outline_colour=000000&coffee_colour=ffffff" /></a>

# ADI Trinamic TMC2208 smart stepper driver

<figure>
  <img src="./docs/header.gif" width=100%>
</figure>

<p align="center">
  <img src="./docs/trinamic-bob-module.jpg" alt="Trinamic BOB" width="19%" />
  <img src="./docs/silentstepstick-module.jpg" alt="SilentStepStick" width="19%" />
  <img src="./docs/bigtreetech-module.webp" alt="BigTreeTech" width="19%" />
  <img src="./docs/fysetc-module.webp" alt="Fysetc" width="19%" />
  <img src="./docs/grobo-module.jpg" alt="GRobotronics" width="19%" />
</p>

# <!-- thin horizontal line -->

</br>

### Table of contents

- [Setup](#setup)
  - [UART](#configuration-of-uart-bus)
  - [Hub](#configuration-of-hub)
  - [Stepper](#stepper-configuration)
    - [Control via UART](#control-the-position-using-serial-uart)
    - [Control via pulses](#control-the-position-using-traditional-stepping-pulses-and-direction)
- [Automation](#automation)
  - [`on_stall`](#on_stall)
  - [`on_status`](#on_status)
- [Actions](#actions)
  - [`tmc2208.configure`](#tmc2208configure-action)
  - [`tmc2208.currents`](#tmc2208currents-action)
  - [`tmc2208.stallguard`](#tmc2208stallguard-action)
  - [`tmc2208.coolconf`](#tmc2208coolconf-action)
  - [`tmc2208.chopconf`](#tmc2208chopconf-action)
  - [`tmc2208.pwmconf`](#tmc2208pwmconf-action)
  - [`tmc2208.enable`](#tmc2208enable-action)
  - [`tmc2208.disable`](#tmc2208disable-action)
  <!-- - [`tmc2208.sync` Action](#tmc2208sync-action) -->
- [Sensors](#sensors)
- [Examples](#example-config)
- [Advanced](#advanced)
- [Wiring](#wiring)
  - [UART Control Wiring](#uart-control)
  - [Pulse Control Wiring](#pulse-control)
- [Resources](#resources)
- [Troubleshooting](#troubleshooting)


<br>

## Setup

Import the component(s).
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ tmc2208_hub, tmc2208, stepper ]
```
---

### Configuration of [UART Bus][uart-component]

```yaml
uart:
  tx_pin: REPLACEME
  rx_pin: REPLACEME
  baud_rate: 500000 # 9600 -> 500k
```

* `baud_rate` (**Required**, int): The baud rate of the UART bus. TMC2208 will auto-detect baud rates from 9600 to 500k with the internal clock/oscillator. An external clock/oscillator is needed for baud rates higher than 500k.

* `tx_pin` (*Required*, [Output Pin Schema][config-pin]): This is the ESPHome device's transmit pin. This should be connected through a 1k Ohm resistor to `PDN_UART` on the TMC2208.

* `rx_pin` (*Required*, [Input Pin Schema][config-pin]): This is the ESPHome device's receive pin. This should be connected directly to `PDN_UART` on the TMC2208.

> [!NOTE]
*Avoid selecting a UART which is utilized for other purposes. For instance boot log as the TMC2208 will try to interpret the output.*

---



### Configuration of hub

This part facilitates a semaphore-like channel to allow multiple drivers on same UART.

> [!TIP]
*Configuration of the hub can be omitted if only a single UART is configured.*

```yaml
tmc2208_hub:

# or with ids
tmc2208_hub:
  id: REPLACEME
  uart_id: REPLACEME

# or multiple hubs
tmc2208_hub:
  - id: REPLACEME
    uart_id: REPLACEME
  - id: REPLACEME
    uart_id: REPLACEME
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the hub so that you can explicitly reference it.

* `uart_id` (**Required**, [ID][config-id]): Reference the UART config.

Example of utilizing the hub with two drivers on same UART.
```yaml
uart:
  - id: tmc_comms
    tx_pin: ...
    rx_pin: ...
    baud_rate: ...

  - id: uart_for_other_stuff
    tx_pin: ...
    rx_pin: ...
    baud_rate: ...


tmc2208_hub:
  id: hub1
  uart_id: tmc_comms


stepper:
  - platform: tmc2208
    id: driver1
    tmc2208_hub_id: hub1
    address: 0x00
    ... all other options

  - platform: tmc2208
    id: driver2
    tmc2208_hub_id: hub1
    address: 0x01
    ... all other options
```

Example of omitting `tmc2208_hub` as which UART to use is inferred.
```yaml
uart:
  tx_pin: ...
  rx_pin: ...
  baud_rate: ...

stepper:
  - platform: tmc2208
    id: driver1
    address: 0x00
    ... all other options

  - platform: tmc2208
    id: driver2
    address: 0x01
    ... all other options
```


----


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
*Configure `step_pin` and `dir_pin` (`index_pin` is optional) for this method. Stay below 8 microstepping for best performance.*


####

```yaml
stepper:
  - platform: tmc2208
    id: driver
    max_speed: 500 steps/s
    acceleration: 2500 steps/s^2
    deceleration: 2500 steps/s^2
    address: 0x00
    rsense: REPLACEME
    vsense: False
    ottrim: 0
    analog_current_scale: False
    clock_frequency: 12MHz
    enn_pin: REPLACEME
    diag_pin: REPLACEME
    index_pin: REPLACEME
    # step_pin: REPLACEME
    # dir_pin: REPLACEME
```
* `id` (**Required**, [ID][config-id]): Specify the ID of the stepper so that you can control it.

* `tmc2208_hub_id` (**Required**, [ID][config-id]): Specify the ID of the hub this stepper is connected to. May be be left out if only a single or no `tmc2208_hub` is configured

* `address` (*Optional*, hex): UART address of the IC. Configured by setting MS1_AD0 or MS2_AD1 high or low. Default is `0x00`.

* `enn_pin` (*Optional*, [Output Pin Schema][config-pin]): Enable not input pin for the driver.
    > *Driver can't be enabled if ENN is left floating. Either configure it if it's actually connected or wire it to GND.*

* `diag_pin` (*Optional*, [Input Pin Schema][config-pin]): Error signaling from the driver.

* `index_pin` (*Optional*, [Input Pin Schema][config-pin]): Serves as stepping feedback from the internal step pulse generator (not serving any purpose if `step_pin` and `dir_pin` is configured).

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

* `analog_current_scale` (*Optional*, boolean): If enabled, VREF input can adjust currents between 0 to IRUN. Defaults to `False` meaning the input is ignored and currents will match settings set by `tmc2208.currents`.

> [!NOTE]
*VREF is often a tiny potentiometer. Setting IRUN to 31 will allow VREF adjustments in the full current range allowed by the sense resistors (rsense). Setting IRUN to 16 narrows that range to ~50%. **IRUN effectively sets the upper limit for what VREF can scale to.***

* `config_dump_include_registers` (*Optional*, boolean): Config dump will display current values in the drivers registers if set. Default is false.

* `clock_frequency` (*Optional*, frequency): Timing reference for all functionalities of the driver. Defaults to 12MHz, which all drivers are factory calibrated to. Only set if using external clock.

* All other from [Base Stepper Component][base-stepper-component]


## Automation

### `on_stall`

Will trigger when a stall is detected. This can be used for sensorless homing. Check the [sensorless homing example](#sensorless-homing).
```yaml
stepper:
  - platform: tmc2208
    id: driver
    ...
    on_stall:
        - logger.log: "Motor stalled!"
        - stepper.stop: driver
```

> [!IMPORTANT]
*Incorrect motor parameters and motion jerk/jolt (quick acceleration/deceleration changes) can lead to false stall events. Play around with the parameters for your specific application.*


### `on_status`
An event is fired whenever a driver warning or error is detected. For instance when the driver overheats.
```yaml
stepper:
  - platform: tmc2208
    id: driver
    ...
    on_status:
      - if:
          condition:
            lambda: return code == tmc2208::OVERTEMPERATURE_PREWARNING;
          then:
            - logger.log: "Driver is about to overheat"
```
> <small>All events in an [example config](#status-events)  for easy copy/paste</small>

#### Driver status codes

Most events is signaling that the driver is in a given state. The majority of events also has a `CLEARED` or similar counterpart signaling that the driver is now not in the given state anymore.

  * `DIAG_TRIGGERED` | `DIAG_TRIGGER_CLEARED`  DIAG output is triggered. Primarily driver errors, but also stall event.
  * `RESET` | `RESET_CLEARED` Driver has been reset since last power on.
  * `DRIVER_ERROR` | `DRIVER_ERROR_CLEARED` A driver error was detected.
  * `CP_UNDERVOLTAGE` | `CP_UNDERVOLTAGE_CLEARED` Undervoltage on chargepump input.
  * `OVERTEMPERATURE_PREWARNING` | `OVERTEMPERATURE_PREWARNING_CLEARED` Driver is warning about increasing temperature.
  * `OVERTEMPERATURE` | `OVERTEMPERATURE_CLEARED` Driver is at critical high temperature and is shutting down.
  * `TEMPERATURE_ABOVE_120C` | `TEMPERATURE_BELOW_120C` Temperature is higher or lower than 120C.
  * `TEMPERATURE_ABOVE_143C` | `TEMPERATURE_BELOW_143C` Temperature is higher or lower than 143C.
  * `TEMPERATURE_ABOVE_150C` | `TEMPERATURE_BELOW_150C` Temperature is higher or lower than 150C.
  * `TEMPERATURE_ABOVE_157C` | `TEMPERATURE_BELOW_157C` Temperature is higher or lower than 157C.
  * `OPEN_LOAD` | `OPEN_LOAD_CLEARED` Open load indicator.
  * `OPEN_LOAD_A` | `OPEN_LOAD_A_CLEARED` Open load indicator phase A.
  * `OPEN_LOAD_B` | `OPEN_LOAD_B_CLEARED`  Open load indicator phase B.
  * `LOW_SIDE_SHORT` | `LOW_SIDE_SHORT_CLEARED` Low side short indicator.
  * `LOW_SIDE_SHORT_A` | `LOW_SIDE_SHORT_A_CLEARED` Low side short indicator phase A.
  * `LOW_SIDE_SHORT_B` | `LOW_SIDE_SHORT_B_CLEARED` Low side short indicator phase B.
  * `GROUND_SHORT` | `GROUND_SHORT_CLEARED` Short to ground indicator.
  * `GROUND_SHORT_A` | `GROUND_SHORT_A_CLEARED` Short to ground indicator phase A.
  * `GROUND_SHORT_B` | `GROUND_SHORT_B_CLEARED` Short to ground indicator phase B.


## Actions

> [!NOTE]
*Registers and fields on the driver persist through an ESPHome device reboot, so previously written states may remain active. They are set to OTP/defaults or cleared when the driver power cycles.*


### `tmc2208.configure` Action
Example of configuring the driver. For instance [`on_boot`](https://esphome.io/components/esphome.html#on-boot).
```yaml
on_...:
  - tmc2208.configure:
      direction: REPLACEME
      microsteps: REPLACME
      interpolation: REPLACEME
      enable_spreadcycle: REPLACEME
      tcool_threshold: REPLACEME
      tpwm_threshold: REPLACEME
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `direction` (*Optional*, string, [templatable][config-templatable]): Effectively inverse the rotational direction. Options are `clockwise` or `counterclockwise` and their abbreviations `cw` or `cww`.

* `microsteps` (*Optional*, int, [templatable][config-templatable]): Microstepping. Possible values are `1`, `2`, `4`, `8`, `16`, `32`, `64`, `128`, `256`.

* `interpolation` (*Optional*, bool, [templatable][config-templatable]): The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for the smoothest motor operation.

* `enable_spreadcycle` (*Optional*, bool, [templatable][config-templatable]): `True` completely disables StealthChop and only uses SpreadCycle, `False` allows use of StealthChop. Defaults to OTP.

* `tcool_threshold` (*Optional*, int, [templatable][config-templatable]): Sets **TCOOLTHRS**

* `tpwm_threshold` (*Optional*, int, [templatable][config-templatable]): Sets **TPWMTHRS**


### `tmc2208.currents` Action
Example of configuring currents and standstill mode.
```yaml
on_...:
  - tmc2208.currents:
      standstill_mode: REPLACEME
      irun: REPLACEME
      run_current: REPLACEME
      ihold: REPLACEME
      hold_current: REPLACEME
      tpowerdown: REPLACEME
      iholddelay: REPLACEME
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `standstill_mode` (*Optional*, [templatable][config-templatable]): Standstill mode for when movement stops. Default is OTP. Available modes are:
  * `normal`: Actively breaks the motor.
  * `freewheeling`: `ihold` and/or `hold_current` must be 0 for true freewheeling. Higher IHOLD enforces high currents thus harder braking effect.
  * `short_coil_ls`: Similar to `freewheeling`, but with motor coils shorted to low side voltage.
  * `short_coil_hs`: Similar to `freewheeling`, but with motor coils shorted to high side voltage.

* `irun` (*Optional*, int, [templatable][config-templatable]): IRUN setting. Must be between 0 and 31.

* `run_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IRUN based on RSense and VSense according to [section 9][datasheet].

* `ihold` (*Optional*, int, [templatable][config-templatable]): IHOLD setting. Must be between 0 and 31.

* `hold_current` (*Optional*, current, [templatable][config-templatable]): Converts a RMS current setting to IHOLD based on RSense and VSense according to [section 9][datasheet].

* `tpowerdown` (*Optional*, int, [templatable][config-templatable]): TPOWERDOWN setting. Must be between 0 and 31.

* `iholddelay` (*Optional*, int, [templatable][config-templatable]): IHOLDDELAY setting. Must be between 0 and 31.


> [!NOTE]
*See [section 1.7][datasheet] for visiual graphs of IRUN, TPOWERDOWN and IHOLDDELAY and IHOLD*



### `tmc2208.stallguard` Action
Example of configuring StallGuard.
```yaml
on_...:
  - tmc2208.stallguard:
      threshold: 50
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `threshold` (*Optional*, int, [templatable][config-templatable]):  Sets **SGTHRS**. Value for the StallGuard4 threshold.


### `tmc2208.coolconf` Action
```yaml
on_...:
  - tmc2208.coolconf:
      seimin: REPLACEME
      semax: REPLACEME
      semin: REPLACEME
      sedn: REPLACEME
      seup: REPLACEME
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `seimin` (*Optional*, int, [templatable][config-templatable]): Sets **SEIMIN**

* `semax` (*Optional*, int, [templatable][config-templatable]): Sets **SEMAX**

* `semin` (*Optional*, int, [templatable][config-templatable]): Sets **SEMIN**

* `sedn` (*Optional*, int, [templatable][config-templatable]): Sets **SEDN**

* `seup` (*Optional*, int, [templatable][config-templatable]): Sets **SEUP**



### `tmc2208.chopconf` Action
```yaml
on_...:
  - tmc2208.chopconf:
      tbl: REPLACEME
      hend: REPLACEME
      hstrt: REPLACEME
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `tbl` (*Optional*, int, [templatable][config-templatable]): Sets CHOPCONF **TBL**

* `hend` (*Optional*, int, [templatable][config-templatable]): Sets CHOPCONF **HEND**

* `hstrt` (*Optional*, int, [templatable][config-templatable]): Sets CHOPCONF **HSTRT**



### `tmc2208.pwmconf` Action
```yaml
on_...:
  - tmc2208.pwmconf:
      lim: REPLACEME
      reg: REPLACEME
      freq: REPLACEME
      ofs: REPLACEME
      autograd: REPLACEME
      autoscale: REPLACEME
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `lim` (*Optional*, int, [templatable][config-templatable]): Sets PWMCONF **PWM_LIM**

* `reg` (*Optional*, int, [templatable][config-templatable]): Sets PWMCONF **PWM_REG**

* `freq` (*Optional*, int, [templatable][config-templatable]): Sets PWMCONF **PWM_FREQ**

* `ofs` (*Optional*, int, [templatable][config-templatable]): Sets PWMCONF **PWM_OFS**

* `autograd` (*Optional*, boolean, [templatable][config-templatable]): Sets PWMCONF **PWM_AUTOGRAD**

* `autoscale` (*Optional*, boolean, [templatable][config-templatable]): Sets PWMCONF **PWM_AUTOSCALE**


### `tmc2208.enable` Action

This uses *TOFF* (sets to 3) if `enn_pin` is not set to enable the driver. *Driver will automatically be enabled if new target is issued.*
```yaml
on_...:
  - tmc2208.enable:
      id: driver
      restore_toff: REPLACEME
```
* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `restore_toff` (*Optional*, boolean, [templatable][config-templatable]): Will attempt to recover TOFF value. Used when no `enn_pin` is configured. Enabled by default.


### `tmc2208.disable` Action

This uses *TOFF* (sets to 0) if `enn_pin` is not set to disable the driver. This will also trigger `stepper.stop`.
```yaml
on_...:
  - tmc2208.disable:
      id: driver
      restore_toff: REPLACEME
```

* `id` (**Required**, [ID][config-id]): Reference to the stepper tmc2208 component. Can be left out if only a single TMC2208 is configured.

* `restore_toff` (*Optional*, boolean, [templatable][config-templatable]): Will attempt to recover TOFF value. Used when no `enn_pin` is configured.

<!-- ### `tmc2208.sync` Action

Action to syncronize configuration settings between multiple drivers. Registers that will be synced are `GSTAT`, `IHOLD_IRUN`, `TPOWERDOWN`, `TPWMTHRS`, `TCOOLTHRS`, `SGTHRS`, `COOLCONF`, `CHOPCONF`, `PWM_CONF`. Fields that will be synced are `OTTRIM`.

```yaml
on_...:
  - tmc2208.sync:
      id: driver1
      to: [driver2, driver3, ...]
```

* `id` (**Required**, [ID][config-id]): Reference to the master tmc2208.

* `to` (**Required**, list, [ID][config-id]): Reference to tmc2208 which settings should be applied to. -->



### Sensors

Some metrics from the driver is exposed as a ready-to-use sensor component.

```yaml
sensor:
  - platform: tmc2208
    type: stallguard_result
    name: Driver stallguard
    update_interval: 250ms

  - platform: tmc2208
    type: motor_load
    name: Motor load
    update_interval: 250ms

  - platform: tmc2208
    type: actual_current
    name: Actual current
    update_interval: 250ms

  - platform: tmc2208
    type: pwm_scale_sum
    name: PWM Scale Sum
    update_interval: 250ms

  - platform: tmc2208
    type: pwm_scale_auto
    name: PWM Scale Auto
    update_interval: 250ms

  - platform: tmc2208
    type: pwm_ofs_auto
    name: PWM OFS Auto
    update_interval: 250ms

  - platform: tmc2208
    type: pwm_grad_auto
    name: PWM Grad Auto
    update_interval: 250ms
```
* `tmc2208_id` (*Optional*, [ID][config-id]): Manually specify the ID of the `stepper.tmc2208` you want to use this sensor.

* `type` (**Required**):
  * `stallguard_result` Stator angle shift detected by the driver.
  * `motor_load` Percentage off stall calculated from StallGuard result and set StallGuard threshold. 100% = stalled.
    * Remeber to configure StallGuard threshold for this to work reliably.
    * The load is calculated by. 510 - SG_RESULT / 510 - SGTHRS * 2 = load coefficient.
  * `actual_current` Active current setting. Either IRUN or IHOLD value.
  * `pwm_scale_sum` Actual PWM duty cycle. This value is used for scaling the values CUR_A and CUR_B read from the sine wave table.
  * `pwm_scale_auto` 9 Bit signed offset added to the calculated PWM duty cycle. This is the result of the automatic amplitude regulation based on current measurement.
  * `pwm_ofs_auto` Automatically determined offset value.
  * `pwm_grad_auto` Automatically determined gradient value.

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
  - platform: tmc2208
    id: driver
    ...
    on_stall:
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

Example of monitoring motor load. It is possible, but not advised.
```yaml
sensor:
  - platform: tmc2208
    type: motor_load
    internal: true
    update_interval: 0s
    filters:
      - sliding_window_moving_average:
          window_size: 10
          send_every: 10
    on_value_range:
      - above: 100.0
        then:
          - stepper.stop: driver
```


## Example config
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ tmc2208_hub, tmc2208, stepper ]

# esp32 or esp8266 config..

wifi:
  ssid: !secret WIFI_SSID
  password: !secret WIFI_PASSWORD

esphome:
  name: actuator
  on_boot:
    - tmc2208.configure:
        microsteps: 8
        interpolation: true
    - tmc2208.stallguard:
        threshold: 50
    - tmc2208.currents:
        standstill_mode: freewheeling
        irun: 16
        ihold: 0
        tpowerdown: 0
        iholddelay: 0

uart:
  tx_pin: 16
  rx_pin: 17
  baud_rate: 500000

stepper:
  - platform: tmc2208
    id: driver
    max_speed: 900 steps/s
    acceleration: 1500 steps/s^2
    deceleration: 500 steps/s^2
    config_dump_include_registers: true
    rsense: 110 mOhm
    vsense: False
    index_pin: 42
    diag_pin: 41
    on_stall:
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
  - platform: tmc2208
    type: motor_load
    name: Motor load
    update_interval: 250ms
```

Partial output of above configuration.
```console
...
[00:00:00][C][tmc2208_hub:013]: TMC2208 Hub:
[00:00:00][C][tmc2208_hub:014]:   Drivers in hub (1):
[00:00:00][C][tmc2208_hub:017]:     Driver with id 'driver' on address 0x00
[00:00:00][C][tmc2208:011]: TMC2208 Stepper:
[00:00:00][C][tmc2208:012]:   Acceleration: 1500 steps/s^2
[00:00:00][C][tmc2208:012]:   Deceleration: 500 steps/s^2
[00:00:00][C][tmc2208:012]:   Max Speed: 900 steps/s
[00:00:00][C][tmc2208:013]:   DIAG Pin: GPIO41
[00:00:00][C][tmc2208:013]:   INDEX Pin: GPIO42
[00:00:00][C][tmc2208:013]:   Address: 0x00
[00:00:00][C][tmc2208:013]:   Detected IC version: 0x21
[00:00:00][C][tmc2208:013]:   Microsteps: 8
[00:00:00][C][tmc2208:013]:   Clock frequency: 12000000 Hz (VACTUAL factor: 0.715256)
[00:00:00][C][tmc2208:013]:   Overtemperature: prewarning = 120C | shutdown = 143C
[00:00:00][C][tmc2208:013]:   Stall detection: DIAG interrupt raises flag
[00:00:00][C][tmc2208:013]:   Status check: disabled
[00:00:00][C][tmc2208:013]:   Currents:
[00:00:00][C][tmc2208:013]:     Limits: 1767 mA
[00:00:00][C][tmc2208:013]:     IRUN: 16 (939 mA)
[00:00:00][C][tmc2208:013]:     IHOLD: 0 (0 mA)
[00:00:00][C][tmc2208:013]:     Additional scaling by VREF is enabled
[00:00:00][C][tmc2208:013]:     VSense: False (high heat dissipation)
[00:00:00][C][tmc2208:013]:     RSense: 0.110 Ohm external sense resistors
[00:00:00][C][tmc2208:013]:   Register dump:
[00:00:00][C][tmc2208:013]:     GCONF:        0x000001E1
[00:00:00][C][tmc2208:013]:     GSTAT:        0x00000001
[00:00:00][C][tmc2208:013]:     IFCNT:        0x00000046
[00:00:00][C][tmc2208:013]:     SLAVECONF:    0x00000000
[00:00:00][C][tmc2208:013]:     OTP_PROG:     0x00000000
[00:00:00][C][tmc2208:013]:     OTP_READ:     0x00000010
[00:00:00][C][tmc2208:013]:     IOIN:         0x2100004C
[00:00:00][C][tmc2208:013]:     FACTORY_CONF: 0x00000010
[00:00:00][C][tmc2208:013]:     IHOLD_IRUN:   0x00001000
[00:00:00][C][tmc2208:013]:     TPOWERDOWN:   0x00000000
[00:00:00][C][tmc2208:013]:     TSTEP:        0x000FFFFF
[00:00:00][C][tmc2208:013]:     TPWMTHRS:     0x00000000
[00:00:00][C][tmc2208:013]:     TCOOLTHRS:    0x00000000
[00:00:00][C][tmc2208:013]:     VACTUAL:      0x00000000
[00:00:00][C][tmc2208:013]:     SGTHRS:       0x00000032
[00:00:00][C][tmc2208:013]:     SG_RESULT:    0x00000000
[00:00:00][C][tmc2208:013]:     COOLCONF:     0x00000000
[00:00:00][C][tmc2208:013]:     MSCNT:        0x00000020
[00:00:00][C][tmc2208:013]:     MSCURACT:     0x00F20030
[00:00:00][C][tmc2208:013]:     CHOPCONF:     0x16010053
[00:00:00][C][tmc2208:013]:     DRV_STATUS:   0xC0000000
[00:00:00][C][tmc2208:013]:     PWM_CONF:     0xC80D0E24
[00:00:00][C][tmc2208:013]:     PWM_SCALE:    0x00050006
[00:00:00][C][tmc2208:013]:     PWM_AUTO:     0x000E003F
...
```


### Advanced

Writing to and reading from registers and register fields from the driver can easily be done with the help of preexisting [helper definitions][tmcapi-tmc2208-hwa] from the underlying TMC-API. A description of the register map can be found under [section 5][datasheet].
> [!IMPORTANT]
***The `tmc2208` component holds a mirror in memory of the values written to the driver. This means write-only registers can still be read with below methods provided they have been written already.***
>
>*Definitions ending in `_MASK` or `_SHIFT` should not be used.*

The `tmc2208` base component exposes four methods:
  * `void write_register(uint8_t address, int32_t value)` write/overwrite an entire register.
  * `int32_t read_register(uint8_t address)` read an entire registers.
  * `void write_field(RegisterField field, uint32_t value)` write/overwrite a register field.
  * `uint32_t read_field(RegisterField field)` read a register field.
  * `uint32_t extract_field(uint32_t data, RegisterField field)` extract field value from register value.

> [!CAUTION]
*Overwriting below registers may cause instability in the ESPHome component and should be avoided.*
  * `VACTUAL_FIELD`
  * `TOFF_FIELD`
  * `VSENSE_FIELD`
  * `DEDGE_FIELD`
  * `INDEX_OTPW_FIELD`
  * `INDEX_STEP_FIELD`
  * `MULTISTEP_FILT_FIELD`
  * `PDN_DISABLE_FIELD`
  * `INTERNAL_RSENSE_FIELD`
  * `I_SCALE_ANALOG_FIELD`
  * `TEST_MODE_FIELD`


Example usage in lambdas
```yaml
sensor:

    // Read stallguard result (register) into a sensor
  - platform: template
    name: Stallguard result
    lambda: return id(driver)->read_register(SG_RESULT);

    // Read microstep selection index into a sensor. This is a binary exponent like 0,1,2,3,... and microsteps can be calculated like 2**<exponent>
  - platform: template
    name: Microstep selection index
    lambda: return id(driver)->read_field(MRES_FIELD);


number:

    // Write value to stallguard threshold register
  - platform: template
    name: Stallguard threshold
    update_interval: 1s
    min_value: 0
    max_value: 255
    step: 5
    lambda: return id(driver)->read_register(SGTHRS);
    set_action:
      - lambda: id(driver)->write_register(SGTHRS, x);


button:

    // Write value 3 to MRES register field. 2**3 = microstepping of 8
  - platform: template
    name: Set microstepping to 8
    on_press:
      - lambda: id(driver)->write_field(MRES_FIELD, 3);

```


## Wiring
Guides to wire ESPHome supported MCU to a TMC2208 driver for either only UART control or pulse control.

### UART Control
Wiring for [UART control](#control-the-position-using-serial-uart)

<img src="./docs/uart-wiring.svg" alt="UART wiring" style="border: 10px solid white" width="100%" />


### Pulse control
Wiring for [Pulse control](#control-the-position-using-traditional-stepping-pulses-and-direction)

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
https://www.analog.com/en/products/tmc2208.html \
https://www.programming-electronics-diy.xyz/2023/12/tmc2208-stepper-driver-module-tutorial.html



## Troubleshooting

#### `Unable to read IC version. Is the driver powered and wired correctly?`
1. Make sure UART is correctly wired and the 1k Ohm resistor is placed correctly.
2. Make sure the driver is power on VM / VS (motor supply voltage). Must be between 4.75 and 29V.

#### `Detected unknown IC version: 0x??`
First generation of TMC2208s have version `0x21`. There is only a single version released as of Q3 2024. If you are seeing version `0x20` that means you have a TMC2208 which is not supported by this component.

#### `Reading from UART timed out at byte 0!`
Poor signal integrity can cause instability in the UART connection. The component doesn't retry writing/reading if a reading failed. Make sure the connection is reliable for best performance. Try lower baud rates if these only appear occasionally.

#### Driver makes "sizzling" noise
Long wires connected to ENN might pick up interference causing the driver to make a sizzling noise if left floating.

#### `undefined reference to vtable`
Source code for components aren't fully loading when adding additional components on ESP-IDF framework with an existing compiled binary. For instance the `motor_load` sensor. Solution is to do a clean build.

#### `Component tmc2208 took a long time for an operation ...`
A lot is happening over serial and low baud rates might cause this warning. Make sure to use the highest baud rate possible. Preferably 500k, which is the highest supported baud rate without external clock.


## TODOs
* Reconfigure driver if driver was power cycled.
* OTTRIM not setting or reading properly.
* Implement hardware timer for step pulse generation.
* Use index for warning if stepper is controlled with step/dir.
* Setup with RX omitted.
* `tmc2208_hub` handle chip select like setup for multiple drivers with same addresses.


## MISC

### Status events
All status events configured for easy copy-pasta.
```yaml
stepper:
  - platform: tmc2208
    id: driver
    ...
    on_status:
      - logger.log:
          format: "Driver is reporting an update! (code %d)"
          args: ["code"]
      - if:
          condition:
            lambda: return code == tmc2208::DIAG_TRIGGERED;
          then:
            - logger.log: DIAG_TRIGGERED
      - if:
          condition:
            lambda: return code == tmc2208::DIAG_TRIGGER_CLEARED;
          then:
            - logger.log: DIAG_TRIGGER_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::RESET;
          then:
            - logger.log: RESET
      - if:
          condition:
            lambda: return code == tmc2208::RESET_CLEARED;
          then:
            - logger.log: RESET_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::DRIVER_ERROR;
          then:
            - logger.log: DRIVER_ERROR
      - if:
          condition:
            lambda: return code == tmc2208::DRIVER_ERROR_CLEARED;
          then:
            - logger.log: DRIVER_ERROR_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::CP_UNDERVOLTAGE;
          then:
            - logger.log: CP_UNDERVOLTAGE
      - if:
          condition:
            lambda: return code == tmc2208::CP_UNDERVOLTAGE_CLEARED;
          then:
            - logger.log: CP_UNDERVOLTAGE_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::OVERTEMPERATURE_PREWARNING;
          then:
            - logger.log: OVERTEMPERATURE_PREWARNING
      - if:
          condition:
            lambda: return code == tmc2208::OVERTEMPERATURE_PREWARNING_CLEARED;
          then:
            - logger.log: OVERTEMPERATURE_PREWARNING_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::OVERTEMPERATURE;
          then:
            - logger.log: OVERTEMPERATURE
      - if:
          condition:
            lambda: return code == tmc2208::OVERTEMPERATURE_CLEARED;
          then:
            - logger.log: OVERTEMPERATURE_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_ABOVE_120C;
          then:
            - logger.log: TEMPERATURE_ABOVE_120C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_BELOW_120C;
          then:
            - logger.log: TEMPERATURE_BELOW_120C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_ABOVE_143C;
          then:
            - logger.log: TEMPERATURE_ABOVE_143C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_BELOW_143C;
          then:
            - logger.log: TEMPERATURE_BELOW_143C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_ABOVE_150C;
          then:
            - logger.log: TEMPERATURE_ABOVE_150C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_BELOW_150C;
          then:
            - logger.log: TEMPERATURE_BELOW_150C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_ABOVE_157C;
          then:
            - logger.log: TEMPERATURE_ABOVE_157C
      - if:
          condition:
            lambda: return code == tmc2208::TEMPERATURE_BELOW_157C;
          then:
            - logger.log: TEMPERATURE_BELOW_157C
      - if:
          condition:
            lambda: return code == tmc2208::OPEN_LOAD;
          then:
            - logger.log: OPEN_LOAD
      - if:
          condition:
            lambda: return code == tmc2208::OPEN_LOAD_CLEARED;
          then:
            - logger.log: OPEN_LOAD_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::OPEN_LOAD_A;
          then:
            - logger.log: OPEN_LOAD_A
      - if:
          condition:
            lambda: return code == tmc2208::OPEN_LOAD_A_CLEARED;
          then:
            - logger.log: OPEN_LOAD_A_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::OPEN_LOAD_B;
          then:
            - logger.log: OPEN_LOAD_B
      - if:
          condition:
            lambda: return code == tmc2208::OPEN_LOAD_B_CLEARED;
          then:
            - logger.log: OPEN_LOAD_B_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::LOW_SIDE_SHORT;
          then:
            - logger.log: LOW_SIDE_SHORT
      - if:
          condition:
            lambda: return code == tmc2208::LOW_SIDE_SHORT_CLEARED;
          then:
            - logger.log: LOW_SIDE_SHORT_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::LOW_SIDE_SHORT_A;
          then:
            - logger.log: LOW_SIDE_SHORT_A
      - if:
          condition:
            lambda: return code == tmc2208::LOW_SIDE_SHORT_A_CLEARED;
          then:
            - logger.log: LOW_SIDE_SHORT_A_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::LOW_SIDE_SHORT_B;
          then:
            - logger.log: LOW_SIDE_SHORT_B
      - if:
          condition:
            lambda: return code == tmc2208::LOW_SIDE_SHORT_B_CLEARED;
          then:
            - logger.log: LOW_SIDE_SHORT_B_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::GROUND_SHORT;
          then:
            - logger.log: GROUND_SHORT
      - if:
          condition:
            lambda: return code == tmc2208::GROUND_SHORT_CLEARED;
          then:
            - logger.log: GROUND_SHORT_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::GROUND_SHORT_A;
          then:
            - logger.log: GROUND_SHORT_A
      - if:
          condition:
            lambda: return code == tmc2208::GROUND_SHORT_A_CLEARED;
          then:
            - logger.log: GROUND_SHORT_A_CLEARED
      - if:
          condition:
            lambda: return code == tmc2208::GROUND_SHORT_B;
          then:
            - logger.log: GROUND_SHORT_B
      - if:
          condition:
            lambda: return code == tmc2208::GROUND_SHORT_B_CLEARED;
          then:
            - logger.log: GROUND_SHORT_B_CLEARED
```



[datasheet]: <./docs/TMC2202_TMC2208_TMC2224_datasheet_rev1.13.pdf> "Datasheet rev 1.09"

[config-id]: <https://esphome.io/guides/configuration-types#config-id> "ESPHome ID Config Schema"
[config-pin]: <https://esphome.io/guides/configuration-types#config-pin-schema> "ESPHome Pin Config Schema"
[config-templatable]: <https://esphome.io/automations/templates#config-templatable> "Templatable configuration"
[config-time]: <https://esphome.io/guides/configuration-types#config-time> "ESPHome Time Config Schema"
[uart-component]: <https://esphome.io/components/uart.html> "ESPHome UART Config"
[base-stepper-component]: <https://esphome.io/components/stepper/#base-stepper-configuration> "ESPHome Base Stepper Component"
[base-sensor-component]: <https://esphome.io/components/sensor/#config-sensor> "ESPHome Base Sensor Component"

[highfrequencylooprequester]: <https://github.com/esphome/esphome/blob/9713458368dfb9fd9aab8016cfe8c85d77b04887/esphome/core/helpers.h#L609> "HighFrequencyLoopRequester class"

[tmcapi-tmc2208-hwa]: <./tmc2208_api_registers.h> "TMC-API TMC2208 Hardware Abstractions"

