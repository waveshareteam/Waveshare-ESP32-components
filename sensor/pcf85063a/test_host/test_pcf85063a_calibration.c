#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pcf85063a.h"

#define MOCK_I2C_ERROR 0x7F01

typedef struct {
    int transmit_calls;
    int receive_calls;
    esp_err_t transmit_result;
    esp_err_t receive_result;
    int transmit_error_call;
    uint8_t transmit_data[8];
    size_t transmit_size;
    uint8_t receive_register;
    uint8_t receive_value;
} mock_i2c_state_t;

static mock_i2c_state_t mock_i2c;
static i2c_master_bus_t mock_bus;
static i2c_master_dev_t mock_device;

#define CHECK_EQ(expected, actual)                                                       \
    do {                                                                                 \
        long expected_value = (long)(expected);                                          \
        long actual_value = (long)(actual);                                              \
        if (expected_value != actual_value) {                                             \
            fprintf(stderr, "%s:%d expected %ld, got %ld\n", __func__, __LINE__,        \
                    expected_value, actual_value);                                        \
            return 1;                                                                    \
        }                                                                                \
    } while (0)

static void mock_i2c_reset(void)
{
    memset(&mock_i2c, 0, sizeof(mock_i2c));
    mock_i2c.transmit_result = ESP_OK;
    mock_i2c.receive_result = ESP_OK;
}

static pcf85063a_dev_t make_device(void)
{
    pcf85063a_dev_t dev = {
        .bus_handle = &mock_bus,
        .dev_handle = &mock_device,
    };
    return dev;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus_handle,
                                    const i2c_device_config_t *dev_config,
                                    i2c_master_dev_handle_t *dev_handle)
{
    (void)bus_handle;
    (void)dev_config;
    *dev_handle = &mock_device;
    return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev_handle,
                              const uint8_t *write_buffer,
                              size_t write_size,
                              int timeout_ms)
{
    (void)dev_handle;
    (void)timeout_ms;
    mock_i2c.transmit_calls++;
    mock_i2c.transmit_size = write_size;
    if (write_size <= sizeof(mock_i2c.transmit_data)) {
        memcpy(mock_i2c.transmit_data, write_buffer, write_size);
    }
    if (mock_i2c.transmit_error_call != 0 &&
        mock_i2c.transmit_calls != mock_i2c.transmit_error_call) {
        return ESP_OK;
    }
    return mock_i2c.transmit_result;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev_handle,
                                      const uint8_t *write_buffer,
                                      size_t write_size,
                                      uint8_t *read_buffer,
                                      size_t read_size,
                                      int timeout_ms)
{
    (void)dev_handle;
    (void)timeout_ms;
    mock_i2c.receive_calls++;
    if (write_size == 1) {
        mock_i2c.receive_register = write_buffer[0];
    }
    if (mock_i2c.receive_result == ESP_OK) {
        memset(read_buffer, mock_i2c.receive_value, read_size);
    }
    return mock_i2c.receive_result;
}

static int test_offset_encoding(void)
{
    static const struct {
        int8_t offset;
        uint8_t encoded;
    } cases[] = {
        {-64, 0x40},
        {-1, 0x7F},
        {0, 0x00},
        {1, 0x01},
        {63, 0x3F},
    };
    static const pcf85063a_offset_mode_t modes[] = {
        PCF85063A_OFFSET_MODE_TWO_HOURS,
        PCF85063A_OFFSET_MODE_FOUR_MINUTES,
    };
    pcf85063a_dev_t dev = make_device();

    for (size_t mode_index = 0; mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
        for (size_t case_index = 0; case_index < sizeof(cases) / sizeof(cases[0]); ++case_index) {
            mock_i2c_reset();
            CHECK_EQ(ESP_OK, pcf85063a_set_offset(&dev, cases[case_index].offset, modes[mode_index]));
            CHECK_EQ(1, mock_i2c.transmit_calls);
            CHECK_EQ(0, mock_i2c.receive_calls);
            CHECK_EQ(2, mock_i2c.transmit_size);
            CHECK_EQ(PCF85063A_RTC_OFFSET_ADDR, mock_i2c.transmit_data[0]);
            CHECK_EQ(cases[case_index].encoded |
                         (modes[mode_index] == PCF85063A_OFFSET_MODE_FOUR_MINUTES ? 0x80 : 0x00),
                     mock_i2c.transmit_data[1]);
        }
    }

    return 0;
}

static int test_default_calibration(void)
{
    pcf85063a_dev_t dev = {0};

    mock_i2c_reset();
    CHECK_EQ(ESP_OK, pcf85063a_init(&dev, &mock_bus, PCF85063A_ADDRESS));
    CHECK_EQ(2, mock_i2c.transmit_calls);
    CHECK_EQ(PCF85063A_RTC_OFFSET_ADDR, mock_i2c.transmit_data[0]);
    CHECK_EQ(0x73, mock_i2c.transmit_data[1]);

    mock_i2c_reset();
    CHECK_EQ(ESP_OK, pcf85063a_reset(&dev));
    CHECK_EQ(2, mock_i2c.transmit_calls);
    CHECK_EQ(PCF85063A_RTC_OFFSET_ADDR, mock_i2c.transmit_data[0]);
    CHECK_EQ(0x73, mock_i2c.transmit_data[1]);

    mock_i2c_reset();
    mock_i2c.transmit_result = MOCK_I2C_ERROR;
    CHECK_EQ(MOCK_I2C_ERROR, pcf85063a_init(&dev, &mock_bus, PCF85063A_ADDRESS));
    CHECK_EQ(1, mock_i2c.transmit_calls);

    mock_i2c_reset();
    mock_i2c.transmit_result = MOCK_I2C_ERROR;
    mock_i2c.transmit_error_call = 2;
    CHECK_EQ(MOCK_I2C_ERROR, pcf85063a_reset(&dev));
    CHECK_EQ(2, mock_i2c.transmit_calls);

    return 0;
}

static int test_offset_validation_and_write_error(void)
{
    pcf85063a_dev_t dev = make_device();
    pcf85063a_dev_t invalid_dev = {0};

    mock_i2c_reset();
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_offset(&dev, -65, PCF85063A_OFFSET_MODE_TWO_HOURS));
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_offset(&dev, 64, PCF85063A_OFFSET_MODE_TWO_HOURS));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_set_offset(&dev, 0, (pcf85063a_offset_mode_t)2));
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_offset(NULL, 0, PCF85063A_OFFSET_MODE_TWO_HOURS));
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_offset(&invalid_dev, 0, PCF85063A_OFFSET_MODE_TWO_HOURS));
    CHECK_EQ(0, mock_i2c.transmit_calls);

    mock_i2c_reset();
    mock_i2c.transmit_result = MOCK_I2C_ERROR;
    CHECK_EQ(MOCK_I2C_ERROR,
             pcf85063a_set_offset(&dev, 1, PCF85063A_OFFSET_MODE_FOUR_MINUTES));
    CHECK_EQ(1, mock_i2c.transmit_calls);

    return 0;
}

static int test_offset_decoding(void)
{
    static const struct {
        uint8_t encoded;
        int8_t offset;
    } cases[] = {
        {0x40, -64},
        {0x7F, -1},
        {0x00, 0},
        {0x01, 1},
        {0x3F, 63},
    };
    pcf85063a_dev_t dev = make_device();

    for (uint8_t mode_bit = 0; mode_bit <= 0x80; mode_bit += 0x80) {
        for (size_t case_index = 0; case_index < sizeof(cases) / sizeof(cases[0]); ++case_index) {
            int8_t offset = 0;
            pcf85063a_offset_mode_t mode = PCF85063A_OFFSET_MODE_TWO_HOURS;

            mock_i2c_reset();
            mock_i2c.receive_value = cases[case_index].encoded | mode_bit;
            CHECK_EQ(ESP_OK, pcf85063a_get_offset(&dev, &offset, &mode));
            CHECK_EQ(0, mock_i2c.transmit_calls);
            CHECK_EQ(1, mock_i2c.receive_calls);
            CHECK_EQ(PCF85063A_RTC_OFFSET_ADDR, mock_i2c.receive_register);
            CHECK_EQ(cases[case_index].offset, offset);
            CHECK_EQ(mode_bit == 0 ? PCF85063A_OFFSET_MODE_TWO_HOURS
                                   : PCF85063A_OFFSET_MODE_FOUR_MINUTES,
                     mode);
        }
        if (mode_bit == 0x80) {
            break;
        }
    }

    return 0;
}

static int test_offset_read_validation_and_error(void)
{
    pcf85063a_dev_t dev = make_device();
    pcf85063a_dev_t invalid_dev = {0};
    int8_t offset = 12;
    pcf85063a_offset_mode_t mode = PCF85063A_OFFSET_MODE_FOUR_MINUTES;

    mock_i2c_reset();
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_offset(NULL, &offset, &mode));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_offset(&dev, NULL, &mode));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_offset(&dev, &offset, NULL));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_offset(&invalid_dev, &offset, &mode));
    CHECK_EQ(0, mock_i2c.receive_calls);

    mock_i2c_reset();
    mock_i2c.receive_result = MOCK_I2C_ERROR;
    CHECK_EQ(MOCK_I2C_ERROR, pcf85063a_get_offset(&dev, &offset, &mode));
    CHECK_EQ(1, mock_i2c.receive_calls);
    CHECK_EQ(12, offset);
    CHECK_EQ(PCF85063A_OFFSET_MODE_FOUR_MINUTES, mode);

    return 0;
}

static int test_load_capacitance_rmw(void)
{
    pcf85063a_dev_t dev = make_device();
    pcf85063a_load_capacitance_t capacitance;

    mock_i2c_reset();
    mock_i2c.receive_value = 0xAD;
    CHECK_EQ(ESP_OK, pcf85063a_set_load_capacitance(&dev, PCF85063A_LOAD_CAPACITANCE_7_PF));
    CHECK_EQ(1, mock_i2c.receive_calls);
    CHECK_EQ(1, mock_i2c.transmit_calls);
    CHECK_EQ(PCF85063A_RTC_CTRL_1_ADDR, mock_i2c.transmit_data[0]);
    CHECK_EQ(0xAC, mock_i2c.transmit_data[1]);

    mock_i2c_reset();
    mock_i2c.receive_value = 0xAC;
    CHECK_EQ(ESP_OK,
             pcf85063a_set_load_capacitance(&dev, PCF85063A_LOAD_CAPACITANCE_12_5_PF));
    CHECK_EQ(0xAD, mock_i2c.transmit_data[1]);

    mock_i2c_reset();
    mock_i2c.receive_value = 0xAC;
    CHECK_EQ(ESP_OK, pcf85063a_get_load_capacitance(&dev, &capacitance));
    CHECK_EQ(PCF85063A_LOAD_CAPACITANCE_7_PF, capacitance);

    mock_i2c_reset();
    mock_i2c.receive_value = 0xAD;
    CHECK_EQ(ESP_OK, pcf85063a_get_load_capacitance(&dev, &capacitance));
    CHECK_EQ(PCF85063A_LOAD_CAPACITANCE_12_5_PF, capacitance);

    return 0;
}

static int test_load_capacitance_validation_and_errors(void)
{
    pcf85063a_dev_t dev = make_device();
    pcf85063a_dev_t invalid_dev = {0};
    pcf85063a_load_capacitance_t capacitance = PCF85063A_LOAD_CAPACITANCE_12_5_PF;

    mock_i2c_reset();
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_load_capacitance(NULL, PCF85063A_LOAD_CAPACITANCE_7_PF));
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_load_capacitance(&dev, (pcf85063a_load_capacitance_t)2));
    CHECK_EQ(ESP_ERR_INVALID_ARG,
             pcf85063a_set_load_capacitance(&invalid_dev, PCF85063A_LOAD_CAPACITANCE_7_PF));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_load_capacitance(NULL, &capacitance));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_load_capacitance(&dev, NULL));
    CHECK_EQ(ESP_ERR_INVALID_ARG, pcf85063a_get_load_capacitance(&invalid_dev, &capacitance));
    CHECK_EQ(0, mock_i2c.receive_calls);
    CHECK_EQ(0, mock_i2c.transmit_calls);

    mock_i2c_reset();
    mock_i2c.receive_result = MOCK_I2C_ERROR;
    CHECK_EQ(MOCK_I2C_ERROR,
             pcf85063a_set_load_capacitance(&dev, PCF85063A_LOAD_CAPACITANCE_7_PF));
    CHECK_EQ(1, mock_i2c.receive_calls);
    CHECK_EQ(0, mock_i2c.transmit_calls);

    mock_i2c_reset();
    mock_i2c.receive_value = 0xAC;
    mock_i2c.transmit_result = MOCK_I2C_ERROR;
    CHECK_EQ(MOCK_I2C_ERROR,
             pcf85063a_set_load_capacitance(&dev, PCF85063A_LOAD_CAPACITANCE_12_5_PF));
    CHECK_EQ(1, mock_i2c.receive_calls);
    CHECK_EQ(1, mock_i2c.transmit_calls);

    mock_i2c_reset();
    mock_i2c.receive_result = MOCK_I2C_ERROR;
    CHECK_EQ(MOCK_I2C_ERROR, pcf85063a_get_load_capacitance(&dev, &capacitance));
    CHECK_EQ(PCF85063A_LOAD_CAPACITANCE_12_5_PF, capacitance);

    return 0;
}

static int run_test(const char *name, int (*test_fn)(void))
{
    int result = test_fn();
    printf("%s: %s\n", name, result == 0 ? "PASS" : "FAIL");
    return result;
}

int main(void)
{
    int failures = 0;

    failures += run_test("offset encoding", test_offset_encoding);
    failures += run_test("default calibration", test_default_calibration);
    failures += run_test("offset validation and write error", test_offset_validation_and_write_error);
    failures += run_test("offset decoding", test_offset_decoding);
    failures += run_test("offset read validation and error", test_offset_read_validation_and_error);
    failures += run_test("load capacitance read-modify-write", test_load_capacitance_rmw);
    failures += run_test("load capacitance validation and errors",
                         test_load_capacitance_validation_and_errors);

    if (failures != 0) {
        fprintf(stderr, "%d host test(s) failed\n", failures);
        return 1;
    }

    printf("All PCF85063A host tests passed\n");
    return 0;
}
