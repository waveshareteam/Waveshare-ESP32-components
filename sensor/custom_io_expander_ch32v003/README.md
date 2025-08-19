# Custom IO Expander Chip CH34V003

[![Component Registry](https://components.espressif.com/components/espressif/esp_io_expander_tca9554/badge.svg)](https://components.espressif.com/components/espressif/esp_io_expander_tca9554)

Implementation of the Custom io expander chip (CH32V003) with esp_io_expander component.

| Chip             | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| CH32V003         | I2C                     | custom_io_expander_ch32v003 | [datasheet](https://www.wch-ic.com/downloads/CH32V003DS0_PDF.html) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.
```
    idf.py add-dependency custom_io_expander_ch32v003==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

Creation of the i2c bus.

```c
    i2c_master_bus_handle_t i2c_handle = NULL;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 8,
        .scl_io_num = 9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    i2c_new_master_bus(&bus_config, &i2c_handle);
```

Creation of the component.

```c
    esp_io_expander_handle_t custom_io_expander = NULL;
    custom_io_expander_new_i2c_ch32v003(i2c_handle, BSP_IO_EXPANDER_I2C_ADDRESS, &custom_io_expander);
```

Print all pins's status to the log:

```c
    esp_io_expander_print_state(custom_io_expander);
```

Set pin 0 and pin 1 with output dircetion and low level:

```c
    esp_io_expander_set_dir(custom_io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(custom_io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);
```

Set pin 2 and pin 3 with input dircetion:

```c
    uint32_t pin_levels = 0;
    esp_io_expander_set_dir(custom_io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, IO_EXPANDER_INPUT);
    esp_io_expander_get_level(custom_io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, &pin_levels);
```
