# TMC2209

```yaml
external_components:
    - source: github://slimcdk/esphome-custom-components
      components: [ tmc2209 ]

esphome:
  ...
  on_boot:
    - tmc2209.configure:
        id: motor
        coolstep_tcoolthrs: 400
        stallguard_sgthrs: 75
        microsteps: 8
        rms_current: 800mA

stepper:
  - platform: tmc2209
    id: motor
    enn_pin:
      number: ...
      inverted: true
    index_pin: ...
    max_speed: 2000 steps/s

    # optional
    acceleration: 2000 steps/s^2
    deceleration: 2000 steps/s^2
    diag_pin: ...
    address: 0x00
    internal_rsense: false
    rsense: 0.11
    on_stall:
      - logger.log: "Motor stalled!"
      - tmc2209.stop: motor
```
