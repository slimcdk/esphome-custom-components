# TMC2209


### Example config
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ tmc2209 ]

uart:
  tx_pin: REPLACEME
  rx_pin: REPLACEME
  baud_rate: 115200

esphome:
  ...
  on_boot:
    - tmc2209.configure:
        id: motor
        coolstep_tcoolthrs: 400
        stallguard_sgthrs: 75
        microsteps: 32
        rms_current: 400mA

stepper:
  - platform: tmc2209
    id: motor
    enn_pin: REPLACEME
    index_pin: REPLACEME
    max_speed: 25000 steps/s

    # optional
    acceleration: 50000 steps/s^2
    deceleration: 50000 steps/s^2
    diag_pin: REPLACEME
    address: 0x00
    rsense: 110 mOhm # set empty to use internal sensing (current limit is set by vref)
    oscillator_freq: 12MHz
    on_stall:
      - logger.log: "Motor stalled!"
      - tmc2209.stop: motor # as of now this is performed from inside the component as well
```
