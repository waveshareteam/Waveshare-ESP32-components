#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_lora_1121.h"

#include "lr1121_config.h"

static const char *TAG = "lr1121_example";

#define TEST_SPI_MOSI             (GPIO_NUM_45)
#define TEST_SPI_MISO             (GPIO_NUM_46)
#define TEST_SPI_CLK              (GPIO_NUM_40)
#define TEST_SPI_CS               (GPIO_NUM_42)
#define TEST_SPI_INT              (GPIO_NUM_38)
#define TEST_SPI_RESET            (GPIO_NUM_39)
#define TEST_SPI_BUSY             (GPIO_NUM_41)

#define TEST_SPI_NUM              (SPI3_HOST)
#define TEST_SPI_CLK_SPEED_HZ     8 * 1000 * 1000

#define RX_TIMEOUT_VALUE 600
#define PING_PONG_PREFIX_SIZE 5
#define SYNC_PACKET_THRESHOLD 64
#define ITERATION_INDEX ( PING_PONG_PREFIX_SIZE + 1 )
#define DELAY_BEFORE_TX_MS 20
#define DELAY_PING_PONG_PACE_MS 200
#define IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )

#define HAL_DBG_TRACE_ARRAY( msg, array, len )                                \
    do                                                                        \
    {                                                                         \
        printf( "%s - (%lu bytes):\n", msg, ( uint32_t ) len ); \
        for( uint32_t i = 0; i < ( uint32_t ) len; i++ )                      \
        {                                                                     \
            if( ( ( i % 16 ) == 0 ) && ( i > 0 ) )                            \
            {                                                                 \
                printf( "\n" );                                 \
            }                                                                 \
            printf( " %02X", array[i] );                        \
        }                                                                     \
        printf( "\n" );                                         \
    } while( 0 );

static spi_device_handle_t spi_handle = NULL;// SPI Handle

static uint8_t buffer_tx[PAYLOAD_LENGTH];
static bool    is_master = true;

static const uint8_t ping_msg[PING_PONG_PREFIX_SIZE] = "PING";
static const uint8_t pong_msg[PING_PONG_PREFIX_SIZE] = "PONG";

static uint8_t  iteration       = 0;
static uint16_t packets_to_sync = 0;

bool irq_flag;
/**
 * @brief Handle reception failure for ping-pong example
 */
static void ping_pong_reception_failure_handling( void );

void lora_irq_process( const void* context, lr11xx_system_irq_mask_t irq_filter_mask );

static void IRAM_ATTR isr(void* arg) {
    irq_flag = true; // Set the interrupt flag
}

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

static void lr1121_test_task(void *arg) {

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

    /* Intializes random number generator */
    srand( 10 );

    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[PING_PONG_PREFIX_SIZE] = ( uint8_t ) 0;
    buffer_tx[ITERATION_INDEX]       = ( uint8_t ) ( iteration );
    for( int i = PING_PONG_PREFIX_SIZE + 1 + 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer_tx[i] = i;
    }

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( &lr1121, buffer_tx, PAYLOAD_LENGTH ) );

    
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( &lr1121, 0 ) );
    while (1)
    {
        if(irq_flag)
            lora_irq_process( &lr1121, IRQ_MASK );
        vTaskDelay(1 / portTICK_PERIOD_MS); // Short delay to control the loop speed
    }
}

void on_tx_done( void )
{
    
    printf( "Sent message %s, iteration %d\n", buffer_tx, iteration );
    vTaskDelay( DELAY_PING_PONG_PACE_MS / portTICK_PERIOD_MS);
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx(
        &lr1121,
        get_time_on_air_in_ms( ) + RX_TIMEOUT_VALUE + rand( ) % 500 ) );  // Random delay to avoid
                                                                          // unwanted synchronization
}

void lora_receive( const void* context, uint8_t* buffer, uint8_t buffer_length, uint8_t* size )
{
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    lr11xx_radio_pkt_status_lora_t  pkt_status_lora;
    lr11xx_radio_pkt_status_gfsk_t  pkt_status_gfsk;

    lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );
    *size = rx_buffer_status.pld_len_in_bytes;
    if( *size > buffer_length )
    {
        printf( "Received payload (size: %d) is bigger than the buffer (size: %d)!\n", *size,
                             buffer_length );
        return;
    }
    lr11xx_regmem_read_buffer8( context, buffer, rx_buffer_status.buffer_start_pointer,
                                rx_buffer_status.pld_len_in_bytes );

    HAL_DBG_TRACE_ARRAY( "Packet content", buffer, *size );

    printf( "Packet status:\n" );
    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
    {
        lr11xx_radio_get_lora_pkt_status( context, &pkt_status_lora );
        printf( "  - RSSI packet = %i dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
        printf( "  - Signal RSSI packet = %i dBm\n", pkt_status_lora.signal_rssi_pkt_in_dbm );
        printf( "  - SNR packet = %i dB\n", pkt_status_lora.snr_pkt_in_db );
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        lr11xx_radio_get_gfsk_pkt_status( context, &pkt_status_gfsk );
        printf( "  - RSSI average = %i dBm\n", pkt_status_gfsk.rssi_avg_in_dbm );
        printf( "  - RSSI sync = %i dBm\n", pkt_status_gfsk.rssi_sync_in_dbm );
    }
}

void on_rx_done( void )
{
    uint8_t buffer_rx[PAYLOAD_LENGTH];
    uint8_t size;

    packets_to_sync = 0;


    lora_receive( &lr1121, buffer_rx, PAYLOAD_LENGTH, &size );
    iteration = buffer_rx[ITERATION_INDEX];

    iteration++;
    printf( "Received message %s, iteration %d\n", buffer_rx, iteration );

    if( is_master == true )
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) == 0 )
        {
            is_master = false;
            memcpy( buffer_tx, pong_msg, PING_PONG_PREFIX_SIZE );
        }
        else if( memcmp( buffer_rx, pong_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            printf( "Unexpected message\n" );
        }
    }
    else
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            printf( "Unexpected message\n" );

            is_master = true;
            memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
        }
    }

    vTaskDelay( (DELAY_PING_PONG_PACE_MS + DELAY_BEFORE_TX_MS) / portTICK_PERIOD_MS);
    buffer_tx[ITERATION_INDEX] = iteration;

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( &lr1121, buffer_tx, PAYLOAD_LENGTH ) );

    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( &lr1121, 0 ) );
}

void on_rx_timeout( void )
{
    packets_to_sync++;
    if( packets_to_sync > SYNC_PACKET_THRESHOLD )
    {
        printf(
            "It looks like synchronisation is still not done, consider resetting one of the board\n" );
    }
    ping_pong_reception_failure_handling( );
}

void on_rx_crc_error( void )
{
    ping_pong_reception_failure_handling( );
}

void on_fsk_len_error( void )
{
    ping_pong_reception_failure_handling( );
}

static void ping_pong_reception_failure_handling( void )
{
    is_master = true;
    iteration = 0;
    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[ITERATION_INDEX] = iteration;

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( &lr1121, buffer_tx, PAYLOAD_LENGTH ) );
    
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( &lr1121, 0 ) );
}

void lora_irq_process( const void* context, lr11xx_system_irq_mask_t irq_filter_mask )
{

    irq_flag = false;

    lr11xx_system_irq_mask_t irq_regs;
    lr11xx_system_get_and_clear_irq_status( context, &irq_regs );

    printf( "Interrupt flags = 0x%08lX\n", irq_regs );

    irq_regs &= irq_filter_mask;

    printf( "Interrupt flags (after filtering) = 0x%08lX\n", irq_regs );

    if( ( irq_regs & LR11XX_SYSTEM_IRQ_TX_DONE ) == LR11XX_SYSTEM_IRQ_TX_DONE )
    {
        printf( "Tx done\n" );
        on_tx_done( );
    }


    if( ( irq_regs & LR11XX_SYSTEM_IRQ_HEADER_ERROR ) == LR11XX_SYSTEM_IRQ_HEADER_ERROR )
    {
        printf( "Header error\n" );
    }

    if( ( irq_regs & LR11XX_SYSTEM_IRQ_RX_DONE ) == LR11XX_SYSTEM_IRQ_RX_DONE )
    {
        if( ( irq_regs & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == LR11XX_SYSTEM_IRQ_CRC_ERROR )
        {
            printf( "CRC error\n" );
            on_rx_crc_error( );
        }
        else if( ( irq_regs & LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR ) == LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )
        {
            printf( "FSK length error\n" );
            on_fsk_len_error( );
        }
        else
        {
            printf( "Rx done\n" );
            on_rx_done( );
        }
    }

    if( ( irq_regs & LR11XX_SYSTEM_IRQ_TIMEOUT ) == LR11XX_SYSTEM_IRQ_TIMEOUT )
    {
        printf( "Rx timeout\n" );
        on_rx_timeout( );
    }

    printf( "\n" );

}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing SPI...");

    esp_err_t ret = spi_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI (error: %d)", ret);
        return;
    }

    xTaskCreate(lr1121_test_task, "lr1121_test_task", 4096, NULL, 5, NULL);
}