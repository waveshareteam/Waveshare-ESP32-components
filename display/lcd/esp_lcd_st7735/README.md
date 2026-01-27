# ESP LCD st7735

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_st7735/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_st7735)

Implementation of the st7735 LCD controller with esp_lcd component.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| st7735         | SPI                     | esp_lcd_st7735     | [PDF1](https://www.waveshare.net/w/upload/e/e2/ST7735S_V1.1_20111121.pdf) |


## Add to project
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency esp_lcd_st7735==1.0.1
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).


## Example use

```c

// add your lcd_init_cmd here
// static st7735_lcd_init_cmd_t st7735_init_cmds[] = {
//     // Rcmd1                                                                                      // Software reset, 150 ms delay
//     {ST7735_SLPOUT, (uint8_t[]){0x00}, 1, 120},                                                                                           // Out of sleep mode, 255 ms delay
//     {ST7735_FRMCTR1, (uint8_t[]){0x05, 0x3C, 0x3C}, 3, 0},                                                                                // Frame rate ctrl - normal mode
//     {ST7735_FRMCTR2, (uint8_t[]){0x05, 0x3C, 0x3C}, 3, 0},                                                                                // Frame rate control - idle mode
//     {ST7735_FRMCTR3, (uint8_t[]){0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C}, 6, 0},                                                              // Frame rate ctrl - partial mode
//     {ST7735_INVCTR, (uint8_t[]){0x03}, 1, 0},                                                                                             // Display inversion ctrl
//     {ST7735_PWCTR1, (uint8_t[]){0xA4, 0x04, 0x84}, 3, 0},                                                                                 // Power control
//     {ST7735_PWCTR2, (uint8_t[]){0xC8}, 1, 0},                                                                                             // Power control
//     {ST7735_PWCTR3, (uint8_t[]){0x0D, 0x00}, 2, 0},                                                                                       // Power control
//     {ST7735_PWCTR4, (uint8_t[]){0x8D, 0x2A}, 2, 0},                                                                                       // Power control
//     {ST7735_PWCTR5, (uint8_t[]){0x8D, 0xEE}, 2, 0},                                                                                       // Power control
//     {ST7735_VMCTR1, (uint8_t[]){0x1D}, 1, 0},                                                                                             // Power control                              
//     {ST7735_GMCTRP1, (uint8_t[]){0x0C, 0x0A, 0x06, 0x00, 0x1A, 0x11, 0x0B, 0x0A, 0x0B, 0x0C, 0x19, 0x33, 0x00, 0x06, 0x02, 0x10}, 16, 0}, // Positive Gamma
//     {ST7735_GMCTRN1, (uint8_t[]){0x0D, 0x0F, 0x03, 0x00, 0x11, 0x0A, 0x06, 0x08, 0x09, 0x0D, 0x1C, 0x3A, 0x00, 0x09, 0x07, 0x10}, 16, 0}, // Negative Gamma
//     {ST7735_TEON, (uint8_t[]){0x00}, 1, 0},
//     {ST7735_COLMOD, (uint8_t[]){0x05}, 1, 0}, 
//     {ST7735_MADCTL, (uint8_t[]){0xC8}, 1, 0},                                                                                 // Normal display on, no args, w/delay 10 ms delay

// };

static esp_err_t lcd_backlight_init(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_GPIO_BL
    };
    return gpio_config(&bk_gpio_config);
}


static esp_err_t lcd_spi_bus_init(void)
{
    const spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_GPIO_SCLK,
        .mosi_io_num = LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,  // ST7735 doesn't need MISO (simplex communication)
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t) 
    };
    return spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO);
}


esp_err_t lcd_init(void)
{
    esp_err_t ret = ESP_OK;
    // 1. Initialize backlight pin
    ESP_GOTO_ON_ERROR(lcd_backlight_init(), err, TAG, "Backlight init failed");

    // 2. Initialize SPI bus
    ESP_LOGD(TAG, "Initialize SPI bus for ST7735");
    ESP_GOTO_ON_ERROR(lcd_spi_bus_init(), err, TAG, "SPI bus init failed");

    // 3. Install LCD panel IO (SPI protocol adaptation)
    ESP_LOGD(TAG, "Install LCD panel IO (SPI)");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_GPIO_DC,
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io), 
                      err, TAG, "Panel IO init failed");


    // st7735_vendor_config_t vendor_config={  
    // .init_cmds = st7735_init_cmds,
    // .init_cmds_size = sizeof(st7735_init_cmds) / sizeof(st7735_lcd_init_cmd_t),
    // };


    // 4. Install ST7735 LCD driver
    ESP_LOGD(TAG, "Install ST7735 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .color_space = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
        // .vendor_config =&vendor_config,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7735(lcd_io, &panel_config, &lcd_panel), 
                      err, TAG, "ST7735 driver init failed");

    // 5. Reset and initialize ST7735 panel
    ESP_LOGD(TAG, "Reset and init ST7735 panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(lcd_panel), err, TAG, "Panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(lcd_panel), err, TAG, "Panel init failed");
    
    // ESP_GOTO_ON_ERROR(esp_lcd_panel_set_gap(lcd_panel,2,3),err, TAG, "Set gap failed");

    // ST7735 has default color inversion; enable color correction
    ESP_GOTO_ON_ERROR(esp_lcd_panel_invert_color(lcd_panel, true), err, TAG, "Invert color failed");
    // Turn on LCD display
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(lcd_panel, true), err, TAG, "Turn on display failed");

    // 6. Turn on LCD backlight
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGD(TAG, "Turn on LCD backlight");
    ESP_GOTO_ON_ERROR(gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL), err, TAG, "Turn on backlight failed");

    ESP_LOGI(TAG, "ST7735 LCD init success");
    return ret;

err:
    // Initialization failed: Release allocated resources to avoid memory leak
    if (lcd_panel != NULL) {
        esp_lcd_panel_del(lcd_panel);
        lcd_panel = NULL;
    }
    if (lcd_io != NULL) {
        esp_lcd_panel_io_del(lcd_io);
        lcd_io = NULL;
    }
    spi_bus_free(LCD_SPI_NUM);
    ESP_LOGE(TAG, "ST7735 LCD init failed (err: %s)", esp_err_to_name(ret));
    return ret;
}
```


