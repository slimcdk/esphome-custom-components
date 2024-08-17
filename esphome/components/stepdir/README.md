# STEPDIR Component

Component to control stepper motor drivers using step pulses and direction signal.

> *Source code for this component is a modified (renamed) version of [A4988](a4988-stepper-component)* \
> All credits to the original authors.


```yaml
stepper:
  - platform: stepdir
    id: motor
    step_pin: REPLACEME
    dir_pin: REPLACEME
    max_speed: 500 steps/s

    # optional
    sleep_pin: REPLACEME
    acceleration: 1000 steps/s^2
    deceleration: 1000 steps/s^2
```

* `id` (**Required**, [ID][config-id]): Specify the ID of the stepper so that you can control it.

* `step_pin` (**Required**, [Pin Schema][config-pin]): The STEP pin of the A4988 stepper driver.

* `dir_pin` (**Required**, [Pin Schema][config-pin]): The DIRECTION pin of the A4988 stepper driver.

* `sleep_pin` (*Optional*, [Pin Schema][config-pin]): Optionally also use the SLEEP pin of the stepper driver. If specified, the driver will be put into sleep mode as soon as the stepper reaches the target steps.

* All other from [Base Stepper Component](base-stepper-component)



## TODO
* Utilize ESP-IDF [MCPWM](espidf-mcpwm) / [PCNT](espidf-pcnt) or [RMT](espidf-rmt) for pulse generation.


[config-id]: <https://esphome.io/guides/configuration-types#config-id> "ESPHome ID Config Schema"
[config-pin]: <https://esphome.io/guides/configuration-types#config-pin-schema> "ESPHome Pin Config Schema"
[base-stepper-component]: <https://esphome.io/components/stepper/#base-stepper-configuration> "Base Stepper Component"
[a4988-stepper-component]: <https://esphome.io/components/stepper/#a4988-component> "A4988 Stepper Component"
[espidf-rmt]: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html> "ESP-IDF RMT"
[espidf-mcpwm]: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html> "ESP-IDF MCPWM"
[espidf-pcnt]: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html> "ESP-IDF PCNT"
