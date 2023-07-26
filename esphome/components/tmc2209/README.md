
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

```
