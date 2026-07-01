# ESP LCD Touch AXS15260D Controller

[![Component Registry](https://components.espressif.com/components/waveshare/esp_lcd_touch_axs15260d/badge.svg)](https://components.espressif.com/components/waveshare/esp_lcd_touch_axs15260d)

Implementation of the AXS15260D touch controller with esp_lcd_touch component.

| Touch controller | Communication interface |     Component name      |                            Link to datasheet                             |
| :--------------: | :---------------------: | :---------------------: | :----------------------------------------------------------------------: |
|    AXS15260D     |          I2C            | esp_lcd_touch_axs15260d |                        []()                                              |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency esp_lcd_touch_axs15260d==0.0.1
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

Initialization of the touch component.

```
    esp_lcd_touch_io_axs15260d_config_t tp_axs15260d_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_AXS15260D_ADDRESS,
    };

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .driver_data = i2c_bus_handle,
    };

    esp_lcd_touch_handle_t tp;
    esp_lcd_touch_new_i2c_axs15260d(io_handle, &tp_cfg, &tp);
```

Read data from the touch controller and store it in RAM memory. It should be called regularly in poll.

```
    esp_lcd_touch_read_data(tp);
```

Get one X and Y coordinates with strength of touch.

```
    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint16_t touch_strength[1];
    uint8_t touch_cnt = 0;

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);
```
