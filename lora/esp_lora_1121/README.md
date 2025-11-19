# LoRa LR1121

[![Component Registry](https://components.espressif.com/components/waveshare/lr1121/badge.svg)](https://components.espressif.com/components/waveshare/lora_1121)

LR1121 transceiver driver, LR1121 is a multi-band, ultra-low-power RF transceiver.

| LoRa controller | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| LR1121           | SPI                     | esp_lora_1121 | [WIKI](https://files.waveshare.com/wiki/Core1121/LR1121_H2_DS_v2_0.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency waveshare/esp_lora_1121==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use
### Initialization of the SPI bus
```c
static esp_err_t spi_bus_init() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = TEST_SPI_MOSI,
        .miso_io_num = TEST_SPI_MISO,
        .sclk_io_num = TEST_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = TEST_SPI_CLK_SPEED_HZ,  // 8 MHz
        .mode = 0,                          // SPI mode 0: CPOL=0, CPHA=0
        .spics_io_num = -1,
        .queue_size = 1,
    };
    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(TEST_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(TEST_SPI_NUM, &devcfg, &spi_handle));
    
    return ESP_OK;
}
```
### LoRa initialization and configuration
```c
    lora_init_io_context(&lr1121,TEST_SPI_CS,TEST_SPI_RESET,TEST_SPI_BUSY,TEST_SPI_INT); // Initialize the I/O context for the LR1121
    lora_init_io(&lr1121);         // Initialize the I/O for the LR1121

    lora_spi_init(&lr1121, spi_handle); // Initialize the SPI interface for the LR1121
    
    printf( "===== LR11xx Ping-Pong example =====\n\n" );
    printf( "LR11XX driver version: %s\n", lr11xx_driver_version_get_version_string( ) );
    
    // Initialize the system
    lora_system_init(&lr1121);
    // Print the version number for verification
    lora_print_version(&lr1121);
    // Initialize the LoRa radio
    lora_radio_init(&lr1121);

    lora_init_irq(&lr1121, isr);     // Initialize the interrupt service routine


    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( &lr1121, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( &lr1121, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

```
### Set the data to be sent and send it.
```c
    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[PING_PONG_PREFIX_SIZE] = ( uint8_t ) 0;
    buffer_tx[ITERATION_INDEX]       = ( uint8_t ) ( iteration );
    for( int i = PING_PONG_PREFIX_SIZE + 1 + 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer_tx[i] = i;
    }

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( &lr1121, buffer_tx, PAYLOAD_LENGTH ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( &lr1121, 0 ) );
```
### Read the current time and check if the alarm has been triggered
```c
    while (1)
    {
        if(irq_flag)
            lora_irq_process( &lr1121, IRQ_MASK );
        vTaskDelay(1 / portTICK_PERIOD_MS); // Short delay to control the loop speed
    }
```

### Running results
```shell
    Interrupt flags = 0x00000004
    Interrupt flags (after filtering) = 0x00000004
    Tx done
    Sent message PING, iteration 50

    Interrupt flags = 0x00000038
    Interrupt flags (after filtering) = 0x00000008
    Rx done
    Packet content - (7 bytes):
    50 4F 4E 47 00 00 33
    Packet status:
    - RSSI packet = -6 dBm
    - Signal RSSI packet = -5 dBm
    - SNR packet = 15 dB
    Received message PONG, iteration 52

    Interrupt flags = 0x00000004
    Interrupt flags (after filtering) = 0x00000004
    Tx done
    Sent message PING, iteration 52
------------
```