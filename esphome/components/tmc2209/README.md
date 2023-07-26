
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components@master
    components: [ tmc2209 ]

esp32:
  board: esp32dev


esphome:
  name: my-stepper

uart:
  tx_pin: 16
  rx_pin: 17
  baud_rate: 115200

stepper:
  - platform: tmc2209
    id: stepper_1
    enable_pin:
      number: 33
      inverted: true
    diag_pin: 2
    index_pin: 4
    address: 0x00
    max_speed: 1000 steps/s
    acceleration: 500 steps/s^2
    deceleration: 500 steps/s^2


text_sensor:
  - platform: template
    name: TMC2209 Version
    lambda: |-
      auto stepper = id(stepper_1);
      auto version = stepper->get_version();
      auto version_hex = str_sprintf("0x%02X", version);
      return {version_hex};
    update_interval: 24h


binary_sensor:
  - platform: template
    name: IOIN ENN
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_enn()};"

  - platform: template
    name: IOIN MS1
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_ms1()};"

  - platform: template
    name: IOIN MS2
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_ms2()};"

  - platform: template
    name: IOIN DIAG
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_diag()};"

  - platform: template
    name: IOIN PDN UART
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_pdn_uart()};"

  - platform: template
    name: IOIN STEP
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_step()};"

  - platform: template
    name: IOIN SPREAD EN
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_spread_en()};"

  - platform: template
    name: IOIN DIR
    lambda: "return {!id(tmc2209_stepper_1)->get_ioin_dir()};"

```
