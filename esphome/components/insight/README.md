# Insight

Sensors which show some internal metrics from the running device. Mostly for debugging.


## Config

Import the component(s).
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ insight ]
```
---


### Sensor


```yaml
sensor
  - platform: insight
    type: esphome_loop
    name: Main loop frequency
    update_interval: 100ms
    filters:
      - sliding_window_moving_average:
          window_size: 10
          send_every: 1
```

* `type` (**Required**):
  * `esphome_loop` Frequency of fastest actual main loop iterations per second.

* All other from [Sensor][base-sensor-component]


[base-sensor-component]: <https://esphome.io/components/sensor/#config-sensor> "ESPHome Base Sensor Component"
