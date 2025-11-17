# Waveshare ESP32-S3-Touch-AMOLED-1.8 Display Support

[![Component Registry](https://components.espressif.com/components/waveshare/esp_oled_ssd1315/badge.svg)](https://components.espressif.com/components/waveshare/esp_oled_ssd1315)

Waveshare ESP32-S3-LR1121-OLED-0.96 Display used I2C

| OLED controller | Communication interface | Component name |                               Link to datasheet                               |
| :------------: |:-----------------------:| :------------: | :---------------------------------------------------------------------------: |
|     SSD1315     |          I2C           | esp_oled_ssd1315 | [PDF](https://files.waveshare.com/upload/f/f0/SSD1315_1.1.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency "waveshare/esp_oled_ssd1315"
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).


## Initialization Code


### I2C Interface

```c
    ESP_LOGI(TAG, "Initialize I2C bus");
    ESP_LOGI(TAG, "I2C Configuration: SDA=GPIO%d, SCL=GPIO%d, Freq=%dHz, Timeout=%dms",
             TEST_PIN_NUM_OLED_SDA, TEST_PIN_NUM_OLED_SCL, TEST_OLED_PIXEL_CLOCK_HZ, TEST_DELAY_TIME_MS);
    ESP_LOGI(TAG, "Display Configuration: Address=0x%02X",
             TEST_OLED_I2C_HW_ADDR);

    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = TEST_OLED_HOST,
        .sda_io_num = TEST_PIN_NUM_OLED_SDA,
        .scl_io_num = TEST_PIN_NUM_OLED_SCL,
        .flags.enable_internal_pullup = true,
    };

    /* Create I2C master bus */
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = TEST_OLED_I2C_HW_ADDR,
        .scl_speed_hz = TEST_OLED_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = TEST_OLED_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = TEST_OLED_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet

    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1315 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = TEST_PIN_NUM_OLED_RST,
    };

    esp_lcd_panel_ssd1315_config_t ssd1315_config = {
        .height = TEST_OLED_V_RES,
    };
    panel_config.vendor_config = &ssd1315_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1315(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle,true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
```

## Notes

