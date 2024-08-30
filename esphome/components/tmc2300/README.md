# TMC2300

```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components@master
    components: [ tmc2300 ]


logger: ...
wifi: ...
api: ...


esp32:
  board: ...
  variant: ...
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_COMPILER_OPTIMIZATION_SIZE: y
    advanced:
      ignore_efuse_mac_crc: true


uart:
  tx_pin: ...
  rx_pin: ...
  baud_rate: 500000


esphome:
  ...
  on_boot:
    - tmc2300.configure:
        id: tmc2300_stepper
        rms_current: 338
        # hold_current_delay: 15
        # coolstep_tcoolthrs: 400
        stallguard_sgthrs: 60
        microsteps: 256


stepper:
  - platform: tmc2300
    id: tmc2300_stepper
    max_speed: 100000 steps/s
    acceleration: 100000 steps/s^2
    deceleration: 100000 steps/s^2
    enn_pin:
      number: ...
      inverted: true
    diag_pin: ...
    address: 0b00
    rsense: 220 mOhm
    internal_rsense: false
    on_stall:
      - logger.log: "Motor stalled!"


sensor:
  - platform: template
    name: Motor load
    lambda: return id(tmc2300_stepper)->motor_load() * 100.0;
    update_interval: 10ms
    internal: true
    filters:
      - sliding_window_moving_average:
          window_size: 10
          send_every: 1


```
