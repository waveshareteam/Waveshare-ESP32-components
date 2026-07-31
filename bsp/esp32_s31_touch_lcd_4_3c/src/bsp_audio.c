/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "bsp_err_check.h"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"

static const char *TAG = "ESP32-S31-Touch-LCD-4.3C";

#define BSP_I2S_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S_MCLK,  \
        .bclk = BSP_I2S_SCLK,  \
        .ws = BSP_I2S_LCLK,    \
        .dout = BSP_I2S_DOUT,  \
        .din = BSP_I2S_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }

#define BSP_I2S_DUPLEX_CFG(_sample_rate)                                                          \
    {                                                                                              \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                       \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                              \
    }

static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;
static i2s_chan_handle_t playback_i2s_tx_chan = NULL;
static const audio_codec_data_if_t *playback_i2s_data_if = NULL;
static const audio_codec_gpio_if_t *playback_gpio_if = NULL;
static const audio_codec_ctrl_if_t *playback_ctrl_if = NULL;
static const audio_codec_if_t *playback_codec_if = NULL;
static esp_codec_dev_handle_t playback_dev_handle = NULL;

static void bsp_audio_release_i2s_channel(i2s_chan_handle_t *channel)
{
    if (channel == NULL || *channel == NULL) {
        return;
    }
    (void)i2s_channel_disable(*channel);
    (void)i2s_del_channel(*channel);
    *channel = NULL;
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    esp_err_t ret = ESP_FAIL;
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = i2s_config ? i2s_config : &std_cfg_default;
    const bool enable_rx = p_i2s_cfg->gpio_cfg.din != I2S_GPIO_UNUSED;
    if (i2s_data_if != NULL) {
        return ESP_OK;
    }
    if (i2s_tx_chan != NULL || i2s_rx_chan != NULL) {
        bsp_audio_release_i2s_channel(&i2s_tx_chan);
        bsp_audio_release_i2s_channel(&i2s_rx_chan);
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_GOTO_ON_ERROR(
        i2s_new_channel(&chan_cfg, &i2s_tx_chan, enable_rx ? &i2s_rx_chan : NULL),
        err, TAG, "I2S channel allocation failed"
    );


    if (i2s_tx_chan != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg), err, TAG, "I2S TX init failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_tx_chan), err, TAG, "I2S TX enable failed");
    }
    if (i2s_rx_chan != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_rx_chan, p_i2s_cfg), err, TAG, "I2S RX init failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "I2S RX enable failed");
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK_GOTO(i2s_data_if, err);

    return ESP_OK;

err:
    bsp_audio_release_i2s_channel(&i2s_tx_chan);
    bsp_audio_release_i2s_channel(&i2s_rx_chan);
    return ret;
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf(void)
{
    return i2s_data_if;
}

static esp_err_t bsp_audio_keep_first_error(esp_err_t current, int next)
{
    return ((current == ESP_OK) && (next != ESP_OK)) ? (esp_err_t)next : current;
}

static esp_err_t bsp_audio_playback_i2s_deinit(void)
{
    esp_err_t ret = ESP_OK;

    if (playback_i2s_data_if) {
        ret = bsp_audio_keep_first_error(ret, audio_codec_delete_data_if(playback_i2s_data_if));
        playback_i2s_data_if = NULL;
    }
    if (playback_i2s_tx_chan) {
        esp_err_t disable_ret = i2s_channel_disable(playback_i2s_tx_chan);
        if (disable_ret != ESP_ERR_INVALID_STATE) {
            ret = bsp_audio_keep_first_error(ret, disable_ret);
        }
        ret = bsp_audio_keep_first_error(ret, i2s_del_channel(playback_i2s_tx_chan));
        playback_i2s_tx_chan = NULL;
    }

    return ret;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_playback_init(void)
{
    if (playback_dev_handle) {
        return playback_dev_handle;
    }
    if (i2s_data_if || i2s_tx_chan || i2s_rx_chan) {
        ESP_LOGE(TAG, "shared audio I2S is already initialized");
        return NULL;
    }

    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    esp_err_t ret = i2s_new_channel(&chan_cfg, &playback_i2s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create playback I2S TX channel failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    const i2s_std_config_t std_cfg = BSP_I2S_DUPLEX_CFG(22050);
    ret = i2s_channel_init_std_mode(playback_i2s_tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "initialize playback I2S TX failed: %s", esp_err_to_name(ret));
        bsp_audio_playback_i2s_deinit();
        return NULL;
    }
    ret = i2s_channel_enable(playback_i2s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "enable playback I2S TX failed: %s", esp_err_to_name(ret));
        bsp_audio_playback_i2s_deinit();
        return NULL;
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = NULL,
        .tx_handle = playback_i2s_tx_chan,
    };
    playback_i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!playback_i2s_data_if) {
        ESP_LOGE(TAG, "create playback I2S data interface failed");
        bsp_audio_playback_i2s_deinit();
        return NULL;
    }

    playback_gpio_if = audio_codec_new_gpio();
    if (!playback_gpio_if) {
        ESP_LOGE(TAG, "create playback codec GPIO interface failed");
        bsp_audio_playback_i2s_deinit();
        return NULL;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = BSP_ES8389_I2C_ADDRESS_8BIT,
        .bus_handle = bsp_i2c_get_handle(),
    };
    playback_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!playback_ctrl_if) {
        ESP_LOGE(TAG, "create playback codec control interface failed");
        audio_codec_delete_gpio_if(playback_gpio_if);
        playback_gpio_if = NULL;
        bsp_audio_playback_i2s_deinit();
        return NULL;
    }

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };
    es8389_codec_cfg_t codec_cfg = {
        .ctrl_if = playback_ctrl_if,
        .gpio_if = playback_gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .no_dac_ref = false,
        .hw_gain = gain,
    };
    playback_codec_if = es8389_codec_new(&codec_cfg);
    if (!playback_codec_if) {
        ESP_LOGE(TAG, "create playback ES8389 interface failed");
        audio_codec_delete_ctrl_if(playback_ctrl_if);
        audio_codec_delete_gpio_if(playback_gpio_if);
        playback_ctrl_if = NULL;
        playback_gpio_if = NULL;
        bsp_audio_playback_i2s_deinit();
        return NULL;
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = playback_codec_if,
        .data_if = playback_i2s_data_if,
    };
    playback_dev_handle = esp_codec_dev_new(&codec_dev_cfg);
    if (!playback_dev_handle) {
        ESP_LOGE(TAG, "create playback codec device failed");
        audio_codec_delete_codec_if(playback_codec_if);
        audio_codec_delete_ctrl_if(playback_ctrl_if);
        audio_codec_delete_gpio_if(playback_gpio_if);
        playback_codec_if = NULL;
        playback_ctrl_if = NULL;
        playback_gpio_if = NULL;
        bsp_audio_playback_i2s_deinit();
    }

    return playback_dev_handle;
}

esp_err_t bsp_audio_codec_speaker_playback_deinit(esp_codec_dev_handle_t play_dev)
{
    if (!playback_dev_handle) {
        return ESP_OK;
    }
    if (play_dev && (play_dev != playback_dev_handle)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = bsp_audio_keep_first_error(ESP_OK, esp_codec_dev_close(playback_dev_handle));
    esp_codec_dev_delete(playback_dev_handle);
    playback_dev_handle = NULL;

    if (playback_codec_if) {
        ret = bsp_audio_keep_first_error(ret, audio_codec_delete_codec_if(playback_codec_if));
        playback_codec_if = NULL;
    }
    if (playback_ctrl_if) {
        ret = bsp_audio_keep_first_error(ret, audio_codec_delete_ctrl_if(playback_ctrl_if));
        playback_ctrl_if = NULL;
    }
    if (playback_gpio_if) {
        ret = bsp_audio_keep_first_error(ret, audio_codec_delete_gpio_if(playback_gpio_if));
        playback_gpio_if = NULL;
    }

    return bsp_audio_keep_first_error(ret, bsp_audio_playback_i2s_deinit());
}
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    const audio_codec_data_if_t *codec_data = bsp_audio_get_codec_itf();
    if (codec_data == NULL) {
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
        codec_data = bsp_audio_get_codec_itf();
    }
    assert(codec_data);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = BSP_ES8389_I2C_ADDRESS_8BIT,
        .bus_handle = bsp_i2c_get_handle(),
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8389_codec_cfg_t codec_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .no_dac_ref = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *dev = es8389_codec_new(&codec_cfg);
    BSP_NULL_CHECK(dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = dev,
        .data_if = codec_data,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    const audio_codec_data_if_t *codec_data = bsp_audio_get_codec_itf();
    if (codec_data == NULL) {
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
        codec_data = bsp_audio_get_codec_itf();
    }
    assert(codec_data);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = BSP_ES8389_I2C_ADDRESS_8BIT,
        .bus_handle = bsp_i2c_get_handle(),
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    es8389_codec_cfg_t codec_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_ADC,
        // Enable both ADCL and ADCR capture channels.
        .no_dac_ref = true,
    };
    const audio_codec_if_t *dev = es8389_codec_new(&codec_cfg);
    BSP_NULL_CHECK(dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = dev,
        .data_if = codec_data,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_duplex_init(void)
{
    const audio_codec_data_if_t *codec_data = bsp_audio_get_codec_itf();
    if (codec_data == NULL) {
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
        codec_data = bsp_audio_get_codec_itf();
    }
    assert(codec_data);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = BSP_ES8389_I2C_ADDRESS_8BIT,
        .bus_handle = bsp_i2c_get_handle(),
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8389_codec_cfg_t codec_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        // Keep the DAC duplex clock/reference path required by this board.
        .no_dac_ref = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *dev = es8389_codec_new(&codec_cfg);
    BSP_NULL_CHECK(dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = dev,
        .data_if = codec_data,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}
