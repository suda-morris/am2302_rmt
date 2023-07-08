# AM2302/DHT22 Example

This example demonstrates how to read the temperature and humidity by using the [am2302_rmt](https://components.espressif.com/component/suda-morris/am2302_rmt) library.

## How to Use Example

### Hardware Required

* A development board with Espressif SoC
* A USB cable for Power supply and programming
* AM2302/DHT22 sensor

### Configure the Example

Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`. Then assign the proper GPIO in the [source file](main/am2302_example_main.c).

### Build and Flash

Run `idf.py -p PORT build flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```text
I (314) main_task: Started on CPU0
I (324) main_task: Calling app_main()
I (324) gpio: GPIO[2]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (334) gpio: GPIO[2]| InputEn: 1| OutputEn: 1| OpenDrain: 1| Pullup: 1| Pulldown: 0| Intr:0
I (344) example: Start reading temperature and humidity from AM2302 sensor
I (2384) example: Temperature: 29.2 °C, Humidity: 89.3 %
I (4414) example: Temperature: 29.2 °C, Humidity: 88.1 %
```
