<a href="https://www.buymeacoffee.com/slimcdk"><img src="https://img.buymeacoffee.com/button-api/?text=Buy me a pizza&emoji=🍕&slug=slimcdk&button_colour=FFDD00&font_colour=000000&font_family=Cookie&outline_colour=000000&coffee_colour=ffffff" /></a>

# Custom components for ESPHome
```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [ <component1>, <component2>, ... ]
```

## Components

- [tmc2209](esphome/components/tmc2209/README.md) :: ADI Trinamic stepper driver.
- [tmc2300](esphome/components/tmc2300/README.md) :: ADI Trinamic stepper driver.
- [tmc5240](esphome/components/tmc5240/README.md) :: ADI Trinamic stepper driver.
- [as5x47](esphome/components/as5x47/README.md) :: quadrature encoder with SPI.
- [freematics](esphome/components/freematics/README.md) :: interface the Freematics firmware.
- [icm20948](esphome/components/icm20948/README.md) :: 9-axis IMU.
- [insight](esphome/components/insight/README.md) :: expose internal metrics from the running device.


## Acknowledgements
This project includes code from the Analog Devices Inc. [TMC-API codebase](https://github.com/analogdevicesinc/TMC-API), licensed under the MIT License.
