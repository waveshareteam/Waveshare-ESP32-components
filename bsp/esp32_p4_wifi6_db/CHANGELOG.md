# Changelog

All notable changes to this component are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this component adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.1] - 2026-09-07

Initial release of the board support package for the Waveshare
ESP32-P4-WIFI6-DB.

### Added

- Shared I2C master bus on GPIO7/GPIO8, with `bsp_i2c_init()`,
  `bsp_i2c_get_handle()` and `bsp_i2c_deinit()`. The ES8311, GT911, display
  backlight controller and camera SCCB controller all use this bus.
- ES8311 audio support: `bsp_audio_init()` plus `bsp_audio_codec_speaker_init()`
  and `bsp_audio_codec_microphone_init()` for playback and for the single
  analog microphone input.
- microSD support over 4-bit SDMMC, including on-chip LDO channel 4 IO power
  and the GPIO45 card power sequence: `bsp_sdcard_mount()`,
  `bsp_sdcard_unmount()`, `bsp_sdcard_sdmmc_mount()` and the
  `bsp_sdcard_get_sdmmc_host()` / `bsp_sdcard_sdmmc_get_slot()` helpers.
- Two-lane MIPI-DSI display with `menuconfig` panel selection between JD9365
  (8 and 10.1 inch), ILI9881C (7 inch) and HX8394 (5 inch), through
  `bsp_display_new()` and `bsp_display_new_with_handles()`.
- Backlight brightness control over I2C address `0x45`, register `0x96`:
  `bsp_display_brightness_set()` / `_get()` and
  `bsp_display_backlight_on()` / `_off()`.
- GT911 touch controller with automatic probing of both I2C addresses
  (`0x5D` and `0x14`), polled because the reset and interrupt pins are not
  connected on this board: `bsp_touch_new()` and `bsp_touch_delete()`.
- LVGL integration through `esp_lvgl_adapter`: `bsp_display_start()`,
  `bsp_display_start_with_config()`, `bsp_display_stop()`,
  `bsp_display_lock()` / `bsp_display_unlock()` and the direct framebuffer
  helpers `bsp_display_set_dummy_draw()`,
  `bsp_display_get_free_frame_buffer()` and
  `bsp_display_flush_frame_buffer()`.
- MIPI-CSI camera initialization through `esp_video`: `bsp_camera_start()`.
- USB Host library lifecycle helpers: `bsp_usb_host_start()` and
  `bsp_usb_host_stop()`.
- SPIFFS mount helpers: `bsp_spiffs_mount()` and `bsp_spiffs_unmount()`.
- Expansion header GPIO enumeration: `bsp_get_header_gpios()`.

### Notes

- The board has no external RTC (`BSP_CAPS_RTC` is 0) and no LCD reset GPIO.
- `bsp_audio_init()` defaults to `BSP_I2S_DEFAULT_SAMPLE_RATE_HZ` (48 kHz),
  mono, duplex, 16-bit when called with `NULL`.
