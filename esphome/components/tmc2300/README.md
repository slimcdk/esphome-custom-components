

```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components@master
    components: [ stepper, tmc2300 ]


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
  tx_pin: 4
  rx_pin: 5
  baud_rate: 500000


esphome:
  ...
  on_boot:
    - lambda: |
        id(tmc2300_stepper).set_microsteps(256);
        id(tmc2300_stepper).stallguard_sgthrs(120);


stepper:
  - platform: tmc2300
    id: tmc2300_stepper
    enn_pin:
      number: ...
      inverted: true
    diag_pin: ...
    index_pin: ...
    max_speed: 100000 steps/s
    acceleration: 150000 steps/s^2
    deceleration: 150000 steps/s^2
    on_fault_signal:
      - logger.log: "Motor stalled!"
```
