# ESP LCD Touch HI8561 Controller

[![Component Registry](https://components.espressif.com/components/waveshare/esp_lcd_touch_hi8561/badge.svg)](https://components.espressif.com/components/waveshare/esp_lcd_touch_hi8561)

I²C driver for the HI8561 touch controller using the `esp_lcd_touch` interface.

## Add to project

This checkout requires ESP-IDF 5.5.5 or later and `espressif/esp_lcd_touch` matching `^1.2.1`.

To use this checkout, add the component directory to your project's `EXTRA_COMPONENT_DIRS` before including `project.cmake` in the top-level `CMakeLists.txt`. For published versions, see the [ESP Component Registry](https://components.espressif.com/components/waveshare/esp_lcd_touch_hi8561).

## Example use

Initialize an I²C master bus with `i2c_new_master_bus()` using your board's SDA and SCL pins. Pass the resulting `i2c_bus` handle through `driver_data`. The driver adds its own device to that bus, so the panel I/O argument can be `NULL`.

```c
#include "esp_lcd_touch_hi8561.h"

static esp_lcd_touch_io_hi8561_config_t hi8561_config;

static esp_err_t init_touch(i2c_master_bus_handle_t i2c_bus, uint16_t width,
                           uint16_t height, esp_lcd_touch_handle_t *touch)
{
    hi8561_config = (esp_lcd_touch_io_hi8561_config_t) {
        .i2c_bus = i2c_bus,
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_HI8561_ADDRESS,
        .scl_speed_hz = 200000,
    };

    const esp_lcd_touch_config_t touch_config = {
        .x_max = width,
        .y_max = height,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .driver_data = &hi8561_config,
    };

    return esp_lcd_touch_new_i2c_hi8561(NULL, &touch_config, touch);
}
```

The example polls for touch and leaves the reset and interrupt pins unassigned. For boards with those pins connected, set the GPIO numbers and the corresponding levels for your hardware. A zero `dev_addr` selects the default 7-bit address (`0x68`). A zero `scl_speed_hz` selects 200 kHz.

After initialization succeeds, call `esp_lcd_touch_read_data()` periodically and retrieve the touch points after a successful read.

```c
esp_lcd_touch_point_data_t points[1];
uint8_t point_count = 0;

if (esp_lcd_touch_read_data(touch) == ESP_OK &&
        esp_lcd_touch_get_data(touch, points, &point_count, 1) == ESP_OK &&
        point_count > 0) {
    // Use points[0].x, points[0].y, and points[0].strength.
}
```

Call `esp_lcd_touch_del(touch)` before deleting the I²C bus. The driver removes its device, and the caller owns the bus.

## Migrate from the previous driver

Replace the I²C device handle previously passed through `user_data` with the I²C master bus handle in `esp_lcd_touch_io_hi8561_config_t.i2c_bus`. Remove `touch_info_start_address` from the configuration.
