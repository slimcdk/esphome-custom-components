external_components:
  - source: github://slimcdk/esphome-custom-components@master
    components: [ tmc2209, gpio ]

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    version: recommended
    sdkconfig_options:
      CONFIG_COMPILER_OPTIMIZATION_SIZE: y
    advanced:
      ignore_efuse_mac_crc: false

esphome:
  name: tmc2209-test

uart:
  tx_pin: 16
  rx_pin: 17
  baud_rate: 115200

binary_sensor:
  - platform: gpio
    name: TMC2209 DIAG
    pin:
      number: 2
      mode:
        input: true
    use_interrupt: true

stepper:
  - platform: tmc2209
    id: tmc2209_stepper
    step_pin: 25
    dir_pin: 32
    enable_pin:
      number: 33
      inverted: true
    # diag_pin: 2
    max_speed: 1000 steps/s
    acceleration: 500 steps/s^2
    deceleration: 500 steps/s^2
