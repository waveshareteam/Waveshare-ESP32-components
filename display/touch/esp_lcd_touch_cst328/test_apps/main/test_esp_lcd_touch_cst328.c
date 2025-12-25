#include "esp_lcd_touch_cst328.h"

#include "lcd_driver.h"
#include "driver/i2c_master.h"
#include "lcd_driver.h"
#include "esp_log.h"


// Pinout配置（保持原定义）
#define I2C_MASTER_SCL_IO   1          /*!< I2C时钟引脚 */
#define I2C_MASTER_SDA_IO   0          /*!< I2C数据引脚 */
#define I2C_MASTER_NUM      I2C_NUM_0   /*!< I2C端口号 */
#define TOUCH_RST      (GPIO_NUM_4)
#define TOUCH_INT      (GPIO_NUM_5)
#define I2C_CLK_SPEED_HZ   400000         /*!< I2C速率 */


static const char *TAG = "Touch Example";

esp_lcd_touch_handle_t tp_handle = NULL;
esp_lcd_panel_io_handle_t tp_io_handle = NULL;
i2c_master_bus_handle_t i2c_handle = NULL;
bool touch_test_done = false;


void touch_init(void)
{
        const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus error");
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

}


//触摸测试  此函数需要传递lcd_init 才可以调用！否则下面有相关宏定义会报错
void touch_test(void)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    uint16_t color_arr[16] = {0};

    for (int i = 0; i < 16; i++)
    {
        color_arr[i] = 0xf800;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
        while (!touch_test_done)
        {
            /* Read data from touch controller into memory */
            esp_lcd_touch_read_data(tp_handle);

            /* Read data from touch controller */
            bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp_handle, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
            if (touchpad_pressed && touchpad_cnt > 0)
            {
                if (touchpad_x[0] < 2)
                    touchpad_x[0] = 2;
                else if (touchpad_x[0] > EXAMPLE_LCD_H_RES - 2 - 1)
                    touchpad_x[0] = EXAMPLE_LCD_H_RES - 2 - 1;

                if (touchpad_y[0] < 2)
                    touchpad_y[0] = 2;
                else if (touchpad_y[0] > EXAMPLE_LCD_V_RES - 2 - 1)
                    touchpad_y[0] = EXAMPLE_LCD_V_RES - 2 - 1;

                esp_lcd_panel_draw_bitmap(panel_handle, touchpad_x[0] - 2, touchpad_y[0] - 2, touchpad_x[0] + 2, touchpad_y[0] + 2, color_arr);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

}

void void app_main(void)
{
    touch_init();
    //lcd _init();
    //touch_test();  //if u want to use this please add lcd_init
}