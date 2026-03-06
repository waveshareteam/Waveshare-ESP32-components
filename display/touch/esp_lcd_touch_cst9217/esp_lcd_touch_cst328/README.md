# ESP LCD Touch CST328 Controller

[![Component Registry](https://components.espressif.com/components/waveshare/esp_lcd_touch_cst328/badge.svg)](https://components.espressif.com/components/waveshare/esp_lcd_touch_cst328)

Implementation of the CST328 touch controller with esp_lcd_touch component.

| Touch controller | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| CST328            | I2C                     | esp_lcd_touch_cst328 | [WIKI](https://www.waveshare.net/wiki/2.8inch_Capacitive_Touch_LCD) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency esp_lcd_touch_cst328==0.0.1
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

Initialization of the touch component.

```

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus error！");
        return;
    }
    ESP_LOGI(TAG, "I2C bus sucessfull");

    i2c_master_bus_handle_t i2c_handle;
    i2c_master_get_bus_handle(0,&i2c_handle);
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = TOUCH_RST,
        .int_gpio_num = TOUCH_INT,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST328_CONFIG();
    tp_io_config.scl_speed_hz = I2C_CLK_SPEED_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((i2c_master_bus_handle_t)i2c_handle, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst328(tp_io_handle, &tp_cfg, &tp_handle));

```
