#include "esp_lora_1121.h"


void lora_init_io_context(const void *context,uint8_t cs,uint8_t reset,uint8_t busy,uint8_t irq)
{
    ((lr1121_t *)context)->cs    = cs;
    ((lr1121_t *)context)->reset = reset;
    ((lr1121_t *)context)->irq   = irq;
    ((lr1121_t *)context)->busy  = busy;
}

void lora_init_io(const void *context)
{
    //Set the output pin
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // Disable interrupts for this pin
    io_conf.pin_bit_mask = 1ULL << ((lr1121_t *)context)->cs | \
                           1ULL << ((lr1121_t *)context)->reset;    // Select the GPIO pin using a bitmask
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;          // Set pin as input
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // Enable internal pull-up resistor
    gpio_config(&io_conf); // Apply the configuration

    //Set the input pin
    io_conf.pin_bit_mask = 1ULL << ((lr1121_t *)context)->busy | \
                           1ULL << ((lr1121_t *)context)->irq;    // Select the GPIO pin using a bitmask
    io_conf.mode = GPIO_MODE_INPUT;          // Set pin as input
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Enable internal pull-up resistor
    gpio_config(&io_conf); // Apply the configuration

    gpio_set_level(((lr1121_t *)context)->cs, 1); // Set the GPIO pin level
    gpio_set_level(((lr1121_t *)context)->reset, 1); // Set the GPIO pin level
}

void lora_init_irq(const void *context, gpio_isr_t handler)
{
    // Zero-initialize the GPIO configuration structure
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;        // Trigger on negative edge (falling edge)
    io_conf.mode = GPIO_MODE_INPUT;               // Set pin as input mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // Enable pull-up resistor
    io_conf.pin_bit_mask = 1ULL << ((lr1121_t *)context)->irq;           // Select the GPIO pin using a bitmask

    gpio_config(&io_conf); // Apply the configuration

    // Install the GPIO interrupt service if not already installed
    gpio_install_isr_service(0); // Pass 0 for default ISR flags

    // Register the interrupt handler for the specified pin
    gpio_isr_handler_add(((lr1121_t *)context)->irq, handler, (void *)((lr1121_t *)context)->irq);
}

void lora_spi_init(const void* context, spi_device_handle_t spi)
{
    ((lr1121_t *)context)->spi = spi;
}

void lora_spi_write_bytes(const void* context,const uint8_t *wirte,const uint16_t wirte_length)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = wirte_length * 8;       // Length is in bits
    t.tx_buffer = wirte;

    ESP_ERROR_CHECK(spi_device_transmit(((lr1121_t *)context)->spi, &t));
}

void lora_spi_read_bytes(const void* context, uint8_t *read,const uint16_t read_length)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = read_length * 8;       // Length is in bits
    t.rx_buffer = read;

    ESP_ERROR_CHECK(spi_device_transmit(((lr1121_t *)context)->spi, &t));
}


lr1121_modem_response_code_t lr1121_modem_board_event_flush( const void* context )
{
    lr1121_modem_response_code_t modem_response_code = LR1121_MODEM_RESPONSE_CODE_OK;
    lr1121_modem_event_fields_t  event_fields;

    do
    {
        modem_response_code = lr1121_modem_get_event( context, &event_fields );
    } while( modem_response_code != LR1121_MODEM_RESPONSE_CODE_NO_EVENT );

    return modem_response_code;
}

