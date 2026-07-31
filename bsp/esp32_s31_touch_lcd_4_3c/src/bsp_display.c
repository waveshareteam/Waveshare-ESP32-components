/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_err_check.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"

static const char *TAG = "ESP32-S31-Touch-LCD-4.3C";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp = NULL;
static lv_indev_t *disp_indev_touch = NULL;
#endif

static bsp_lcd_handles_t disp_handles = {0};
static esp_lcd_touch_handle_t tp = NULL;
static int brightness_percent = 0;
static bool brightness_initialized = false;
static bool gt911_address_selected = false;

#define BSP_BACKLIGHT_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define BSP_BACKLIGHT_LEDC_TIMER      LEDC_TIMER_0
#define BSP_BACKLIGHT_LEDC_CHANNEL    LEDC_CHANNEL_0
#define BSP_BACKLIGHT_MAX_DUTY        (1023U)

#if defined(CONFIG_BSP_DISPLAY_BRIGHTNESS_INVERTED)
#define BSP_BACKLIGHT_INACTIVE_LEVEL  1
#else
#define BSP_BACKLIGHT_INACTIVE_LEVEL  0
#endif

static esp_err_t bsp_lcd_panel_disp_on_off(esp_lcd_panel_handle_t panel, bool on)
{
    esp_err_t ret = esp_lcd_panel_disp_on_off(panel, on);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGD(TAG, "LCD panel disp_on_off(%d) is not supported", on);
        return ESP_OK;
    }
    return ret;
}

static esp_err_t bsp_i2c_device_probe(uint8_t addr)
{
    return i2c_master_probe(bsp_i2c_get_handle(), addr, 100);
}

static uint32_t brightness_to_duty(int brightness)
{
    if (brightness > 100) {
        brightness = 100;
    } else if (brightness < 0) {
        brightness = 0;
    }

#if defined(CONFIG_BSP_DISPLAY_BRIGHTNESS_INVERTED)
    return (uint32_t)((100 - brightness) * BSP_BACKLIGHT_MAX_DUTY / 100);
#else
    return (uint32_t)(brightness * BSP_BACKLIGHT_MAX_DUTY / 100);
#endif
}

esp_err_t bsp_display_brightness_init(void)
{
    if (brightness_initialized) {
        return ESP_OK;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = BSP_BACKLIGHT_LEDC_MODE,
        .timer_num = BSP_BACKLIGHT_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = CONFIG_BSP_DISPLAY_BRIGHTNESS_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "backlight LEDC timer config failed");

    brightness_percent = CONFIG_BSP_DISPLAY_BOOT_BRIGHTNESS_PERCENT;
    const ledc_channel_config_t channel_config = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = BSP_BACKLIGHT_LEDC_MODE,
        .channel = BSP_BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BSP_BACKLIGHT_LEDC_TIMER,
        .duty = brightness_to_duty(brightness_percent),
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "backlight LEDC channel config failed");

    brightness_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_display_brightness_deinit(void)
{
    if (!brightness_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ledc_stop(BSP_BACKLIGHT_LEDC_MODE, BSP_BACKLIGHT_LEDC_CHANNEL,
                                  BSP_BACKLIGHT_INACTIVE_LEVEL),
                        TAG, "backlight LEDC stop failed");
    ESP_RETURN_ON_ERROR(gpio_reset_pin(BSP_LCD_BACKLIGHT), TAG, "backlight GPIO reset failed");
    brightness_initialized = false;
    brightness_percent = 0;
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness)
{
    if (brightness > 100) {
        brightness = 100;
    } else if (brightness < 0) {
        brightness = 0;
    }

    if (!brightness_initialized) {
        ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "backlight init failed");
    }

    const uint32_t duty = brightness_to_duty(brightness);
    ESP_RETURN_ON_ERROR(ledc_set_duty(BSP_BACKLIGHT_LEDC_MODE, BSP_BACKLIGHT_LEDC_CHANNEL, duty),
                        TAG, "backlight duty set failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(BSP_BACKLIGHT_LEDC_MODE, BSP_BACKLIGHT_LEDC_CHANNEL),
                        TAG, "backlight duty update failed");

    brightness_percent = brightness;
    return ESP_OK;
}

int bsp_display_brightness_get(void)
{
    return brightness_percent;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_set_display_pclk(uint32_t freq_hz)
{
    BSP_NULL_CHECK(disp_handles.panel, ESP_ERR_INVALID_STATE);
    return esp_lcd_rgb_panel_set_pclk(disp_handles.panel, freq_hz);
}

static esp_err_t bsp_gt911_select_address_sequence(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_LCD_TOUCH_RST, 0), TAG, "touch reset low failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_LCD_TOUCH_INT, 0), TAG, "touch INT low failed");
    const gpio_config_t output_config = {
        .pin_bit_mask = BIT64(BSP_LCD_TOUCH_RST) | BIT64(BSP_LCD_TOUCH_INT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG, "touch reset/INT output config failed");
    /* GT911 samples INT during reset release: INT low selects 0x5d, INT high selects 0x14. */
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BSP_TOUCH_RESET_HOLD_MS));
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_LCD_TOUCH_RST, 1), TAG, "touch reset high failed");
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BSP_TOUCH_STARTUP_DELAY_MS));

    const gpio_config_t int_input_config = {
        .pin_bit_mask = BIT64(BSP_LCD_TOUCH_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_input_config), TAG, "touch INT input config failed");
    gt911_address_selected = true;

    return ESP_OK;
}

static esp_err_t bsp_gt911_probe(esp_lcd_panel_io_i2c_config_t *tp_io_config)
{
    BSP_NULL_CHECK(tp_io_config, ESP_ERR_INVALID_ARG);

    esp_lcd_panel_io_i2c_config_t config_default = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    if (ESP_OK == bsp_i2c_device_probe(BSP_TOUCH_I2C_ADDRESS)) {
        ESP_LOGI(TAG, "GT911 touch found at 0x%02x", BSP_TOUCH_I2C_ADDRESS);
        memcpy(tp_io_config, &config_default, sizeof(config_default));
        return ESP_OK;
    }

    if (ESP_OK == bsp_i2c_device_probe(BSP_TOUCH_I2C_ADDRESS_ALT)) {
        ESP_LOGI(TAG, "GT911 touch found at 0x%02x", BSP_TOUCH_I2C_ADDRESS_ALT);
        config_default.dev_addr = BSP_TOUCH_I2C_ADDRESS_ALT;
        memcpy(tp_io_config, &config_default, sizeof(config_default));
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    bsp_lcd_handles_t handles = {0};
    esp_err_t ret = bsp_display_new_with_handles(config, &handles);
    if (ret == ESP_OK) {
        if (ret_panel) {
            *ret_panel = handles.panel;
        }
        if (ret_io) {
            *ret_io = handles.io;
        }
    }

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    BSP_NULL_CHECK(ret_handles, ESP_ERR_INVALID_ARG);
    memset(ret_handles, 0, sizeof(*ret_handles));

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_touch_reset(), TAG, "touch reset failed");

    esp_lcd_rgb_panel_config_t panel_conf = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = BSP_DISPLAY_PANEL_RGB_TIMING(),
        .data_width = BSP_RGB_DATA_WIDTH,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB565,
#else
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
#endif
        .num_fbs = CONFIG_BSP_LCD_RGB_BUFFER_NUMS,
        .dma_burst_size = 64,
        .hsync_gpio_num = BSP_LCD_RGB_HSYNC,
        .vsync_gpio_num = BSP_LCD_RGB_VSYNC,
        .de_gpio_num = BSP_LCD_RGB_DE,
        .pclk_gpio_num = BSP_LCD_RGB_PCLK,
        .disp_gpio_num = BSP_LCD_RGB_DISP,
        .data_gpio_nums = {
            BSP_LCD_RGB_DATA0,
            BSP_LCD_RGB_DATA1,
            BSP_LCD_RGB_DATA2,
            BSP_LCD_RGB_DATA3,
            BSP_LCD_RGB_DATA4,
            BSP_LCD_RGB_DATA5,
            BSP_LCD_RGB_DATA6,
            BSP_LCD_RGB_DATA7,
            BSP_LCD_RGB_DATA8,
            BSP_LCD_RGB_DATA9,
            BSP_LCD_RGB_DATA10,
            BSP_LCD_RGB_DATA11,
            BSP_LCD_RGB_DATA12,
            BSP_LCD_RGB_DATA13,
            BSP_LCD_RGB_DATA14,
            BSP_LCD_RGB_DATA15,
        },
#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
        .bounce_buffer_size_px = BSP_LCD_H_RES * CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT,
#endif
        .flags.fb_in_psram = 1,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(&panel_conf, &ret_handles->panel), TAG, "RGB panel init failed");
    disp_handles.panel = ret_handles->panel;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(ret_handles->panel), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(ret_handles->panel), TAG, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(ret_handles->panel, false), TAG, "panel invert failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(ret_handles->panel, false), TAG, "panel swap failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(ret_handles->panel, false, false), TAG, "panel mirror failed");

    ESP_LOGI(TAG, "Display initialized with resolution %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);
    return ESP_OK;
}

void bsp_display_delete(void)
{
    bsp_display_backlight_off();

    if (disp_handles.panel) {
        esp_lcd_panel_del(disp_handles.panel);
        disp_handles.panel = NULL;
    }
    if (disp_handles.io) {
        esp_lcd_panel_io_del(disp_handles.io);
        disp_handles.io = NULL;
    }

    bsp_display_brightness_deinit();
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    BSP_NULL_CHECK(ret_touch, ESP_ERR_INVALID_ARG);
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    const bool swap_xy = config ? config->swap_xy : false;
    const bool mirror_x = config ? config->mirror_x : false;
    const bool mirror_y = config ? config->mirror_y : false;

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = swap_xy,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
    };

    esp_lcd_panel_io_i2c_config_t tp_io_config = {0};
    esp_err_t probe_ret = ESP_ERR_NOT_FOUND;
    for (int attempt = 1; attempt <= CONFIG_BSP_TOUCH_INIT_RETRY_COUNT; attempt++) {
        if (!gt911_address_selected || attempt > 1) {
            gt911_address_selected = false;
            probe_ret = bsp_touch_reset();
            if (probe_ret != ESP_OK) {
                ESP_LOGW(TAG, "GT911 reset/address attempt %d/%d failed: %s",
                         attempt, CONFIG_BSP_TOUCH_INIT_RETRY_COUNT, esp_err_to_name(probe_ret));
                if (CONFIG_BSP_TOUCH_INIT_RETRY_DELAY_MS > 0) {
                    vTaskDelay(pdMS_TO_TICKS(CONFIG_BSP_TOUCH_INIT_RETRY_DELAY_MS));
                }
                continue;
            }
        }

        probe_ret = bsp_gt911_probe(&tp_io_config);
        if (probe_ret == ESP_OK) {
            break;
        }

        ESP_LOGW(TAG, "GT911 touch probe attempt %d/%d failed: %s",
                 attempt, CONFIG_BSP_TOUCH_INIT_RETRY_COUNT, esp_err_to_name(probe_ret));
        gt911_address_selected = false;
        if (CONFIG_BSP_TOUCH_INIT_RETRY_DELAY_MS > 0) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_BSP_TOUCH_INIT_RETRY_DELAY_MS));
        }
    }
    if (probe_ret != ESP_OK) {
        ESP_LOGE(TAG, "GT911 touch controller not found after %d attempts", CONFIG_BSP_TOUCH_INIT_RETRY_COUNT);
        return probe_ret;
    }
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bsp_i2c_get_handle(), &tp_io_config, &tp_io_handle), TAG, "touch IO init failed");

    esp_err_t ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
    if (ret != ESP_OK) {
        esp_lcd_panel_io_del(tp_io_handle);
    }
    return ret;
}

esp_err_t bsp_touch_reset(void)
{
    return bsp_gt911_select_address_sequence();
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);

    const bsp_display_config_t bsp_disp_cfg = {
        .dummy = 0,
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(&bsp_disp_cfg, &disp_handles));
    BSP_ERROR_CHECK_RETURN_NULL(bsp_lcd_panel_disp_on_off(disp_handles.panel, true));

    const uint32_t buffer_height = (cfg->buffer_height > 0) ? cfg->buffer_height : CONFIG_BSP_LCD_DRAW_BUF_HEIGHT;
    const esp_lv_adapter_display_config_t display_config = {
        .panel = disp_handles.panel,
        .panel_io = disp_handles.io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_RGB,
            .rotation = cfg->rotation,
            .hor_res = BSP_LCD_H_RES,
            .ver_res = BSP_LCD_V_RES,
            .buffer_height = buffer_height,
            .use_psram = cfg->use_psram,
            .enable_ppa_accel = cfg->enable_ppa_accel,
            .require_double_buffer = cfg->require_double_buffer,
        },
        .tear_avoid_mode = cfg->tear_avoid_mode,
    };

    return esp_lv_adapter_register_display(&display_config);
}

static lv_indev_t *bsp_display_indev_touch_init(const bsp_display_cfg_t *cfg, lv_display_t *display)
{
    assert(cfg != NULL);

    const bsp_touch_config_t touch_cfg = {
        .swap_xy = cfg->touch_flags.swap_xy,
        .mirror_x = cfg->touch_flags.mirror_x,
        .mirror_y = cfg->touch_flags.mirror_y,
    };
    esp_err_t ret = bsp_touch_new(&touch_cfg, &tp);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Touch init skipped: %s", esp_err_to_name(ret));
        return NULL;
    }
    assert(tp);

    const esp_lv_adapter_touch_config_t lvgl_touch_cfg = {
        .disp = display,
        .handle = tp,
        .scale = {
            .x = 1.0f,
            .y = 1.0f,
        },
    };

    return esp_lv_adapter_register_touch(&lvgl_touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    const bsp_display_cfg_t cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT,
        .buffer_height = CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
        .use_psram = true,
        .enable_ppa_accel = false,
        .require_double_buffer = true,
        .touch_flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(esp_lv_adapter_init(&cfg->lv_adapter_cfg));
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    disp_indev_touch = bsp_display_indev_touch_init(cfg, disp);
    if (disp_indev_touch == NULL) {
        ESP_LOGW(TAG, "Display started without touch input");
    }
    BSP_ERROR_CHECK_RETURN_NULL(esp_lv_adapter_start());

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev_touch;
}

esp_lcd_panel_handle_t bsp_display_get_panel_handle(void)
{
    return disp_handles.panel;
}

void bsp_display_rotate(lv_display_t *display, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(display, rotation);
}

bool bsp_display_lock(int32_t timeout_ms)
{
    const int32_t adapter_timeout_ms = (timeout_ms == 0) ? -1 : timeout_ms;
    return esp_lv_adapter_lock(adapter_timeout_ms) == ESP_OK;
}

void bsp_display_unlock(void)
{
    esp_lv_adapter_unlock();
}

static esp_err_t bsp_lcd_enter_sleep(void)
{
    BSP_NULL_CHECK(disp_handles.panel, ESP_ERR_INVALID_STATE);
    return bsp_lcd_panel_disp_on_off(disp_handles.panel, false);
}

static esp_err_t bsp_lcd_exit_sleep(void)
{
    BSP_NULL_CHECK(disp_handles.panel, ESP_ERR_INVALID_STATE);
    return bsp_lcd_panel_disp_on_off(disp_handles.panel, true);
}

esp_err_t bsp_display_enter_sleep(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_enter_sleep());
    if (tp) {
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_enter_sleep(tp));
    }
    BSP_ERROR_CHECK_RETURN_ERR(bsp_display_backlight_off());
    return ESP_OK;
}

esp_err_t bsp_display_exit_sleep(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_exit_sleep());
    if (tp) {
        BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_exit_sleep(tp));
    }
    BSP_ERROR_CHECK_RETURN_ERR(bsp_display_backlight_on());
    return ESP_OK;
}
#endif
