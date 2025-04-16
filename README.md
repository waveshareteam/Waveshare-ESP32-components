# Waveshare ESP32 Board Support Packages and Drivers

Welcome to the repository for Waveshare ESP32 products! This repository is designed to provide resources, drivers, and
support packages for Waveshare's ESP32-based boards and compatible display modules. It aims to simplify development and
accelerate your project setup.

---

## 📦 Repository Contents

### 1. **Board Support Packages (BSP)**

| Board                                                                                                                                                                                                                                                                                                         | supported |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [ESP32-P4-NANO](https://www.waveshare.com/esp32-p4-nano.htm)<br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/e/s/esp32-p4-nano-1.jpg">                               | ✅         |
| [ESP32-S3-Touch-AMOLED-1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm)<br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/e/s/esp32-s3-touch-amoled-1.8-1.jpg">                          | ✅        |
| [ESP32-S3-Touch-LCD-1.69](https://www.waveshare.com/esp32-s3-touch-lcd-1.69.htm)<br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/e/s/esp32-s3-touch-lcd-1.69-1.jpg"> | 🕒        |
| [ESP32-S3-LCD-1.69](https://www.waveshare.com/esp32-s3-lcd-1.69.htm)<br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/e/s/esp32-s3-lcd-1.69-1.jpg">                   | 🕒        |
| [ESP32-S3-Touch-LCD-4 ](https://www.waveshare.com/esp32-s3-touch-lcd-4.htm)<br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/e/s/esp32-s3-touch-lcd-4-1.jpg">         | ✅        |

### 2. **Display Drivers**

#### Recommended display screen

| Product ID                                                                                                                                                                                                                                                                                               | Dependency                                                       | tested |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|--------|
| [10.1-DSI-TOUCH-A](https://www.waveshare.com/10.1-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1-dsi-touch-a-1.jpg"> | [waveshare/esp_lcd_jd9365_10_1](display/lcd/esp_lcd_jd9365_10_1) | ✅      |
| [101M-8001280-IPS-CT-K](https://www.waveshare.com/101m-8001280-ips-ct-k.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/101m-8001280-ips-ct-k-1.jpg"> | [waveshare/esp_lcd_jd9365_10_1](display/lcd/esp_lcd_jd9365_10_1) | ✅      |
| [7-DSI-TOUCH-A](https://www.waveshare.com/7-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/-/7-dsi-touch-a-1.jpg"> | [waveshare/esp_lcd_ili9881c](display/lcd/esp_lcd_ili9881c)    | ✅      |

#### Common Raspberry adapter screen


<details open>
<summary>View full display</summary>

| Product ID                                                                                                                                                                                                                                                                                               | Dependency                                                       | tested |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|--------|
| [2.8inch DSI LCD](https://www.waveshare.com/2.8inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/thumbnail/122x122/9df78eab33525d08d6e5fb8d27136e95/2/_/2.8inch-dsi-lcd-3.jpg">               | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [3.4inch DSI LCD (C)](https://www.waveshare.com/3.4inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/3/_/3.4inch-dsi-lcd-c-1.jpg">           | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [4inch DSI LCD (C)](https://www.waveshare.com/4inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/i/4inch-dsi-lcd-c-1.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [4inch DSI LCD](https://www.waveshare.com/4inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/i/4inch-dsi-lcd-1.jpg">                         | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [5inch DSI LCD (D)](https://www.waveshare.com/5inch-dsi-lcd-d.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/i/5inch-dsi-lcd-d-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [6.25inch DSI LCD](https://www.waveshare.com/6.25inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/6/_/6.25inch-dsi-lcd-2.jpg">                | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [5inch DSI LCD (C)](https://www.waveshare.com/5inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/i/5inch-dsi-lcd-c-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [7inch DSI LCD (C)](https://www.waveshare.com/7inch-dsi-lcd-c-with-case-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/i/7inch-dsi-lcd-c-4.jpg">     | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [7.9inch DSI LCD](https://www.waveshare.com/7.9inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/_/7.9inch-dsi-lcd-2.jpg">                   | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [7inch DSI LCD (E)](https://www.waveshare.com/7inch-dsi-lcd-e.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/i/7inch-dsi-lcd-e-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [8inch DSI LCD (C)](https://www.waveshare.com/8inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/i/8inch-dsi-lcd-c-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [10.1inch DSI LCD (C)](https://www.waveshare.com/10.1inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1inch-dsi-lcd-c-2.jpg">        | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [8.8inch DSI LCD](https://www.waveshare.com/8.8inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/_/8.8inch-dsi-lcd-2.jpg">                   | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [11.9inch DSI LCD](https://www.waveshare.com/11.9inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/1/11.9inch-dsi-lcd-3.jpg">                | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
</details>

## 📚 Documentation

Each library and support package comes with detailed documentation and examples. Key highlights include:

- **Initialization Guides:** Step-by-step instructions for setting up your board and peripherals.
- **Examples:**
    - The driver library code provides the simplest low-level drivers
    - BSP sample programs can be queried in different product wikis, and will be synchronized in
      the [ESP32-display-support](https://github.com/waveshareteam/ESP32-display-support)
- **Code Comments:** Inline documentation for better understanding.

---

## 🚀 Getting Started

### Prerequisites

- **Hardware:**
    - Waveshare ESP32 boards
    - Supported display modules
    - Compatible touch controllers(Optional)
- **Software:**
    - [ESP-IDF](https://github.com/espressif/esp-idf)(Each product has a recommended version on the wiki as well as
      sample routines)

[//]: # (### Setup)

[//]: # ()

[//]: # (1. Clone the repository:)

[//]: # ()

[//]: # ()

[//]: # (2. Install required dependencies:)

[//]: # ()

[//]: # ()

[//]: # (3. Flash the example projects:)


---

## 🌟 Features

- **LVGL Support:**
    - Widgets, animations, and custom UI components for Waveshare displays.
- **Optimized Drivers:**
    - DMA and double buffering for smooth graphics.
- **Peripheral Utilities:**
    - Simplified access to RTC, I/O expanders, and power management ICs.
- **Arduino-Compatible Libraries:**
    - Easy porting of code using the Arduino ecosystem.

---

## 🔧 Configuration

You can view some of the configurations that have been added via menuconfig, which will allow you to verify the
functionality directly

---

## 📂 Directory Structure

```
.
├── bsp/           # Board support packages
├── display/          # Display drivers
└── README.md         # Project overview (this file)
```

---

## 🛠️ Contributing

We welcome contributions! Here’s how you can help:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Commit your changes with clear descriptions.
4. Submit a pull request for review.

---

## 🧩 Issues and Support

If you encounter any issues:

- Check the [Issues](https://github.com/waveshareteam/Waveshare-ESP32-components/issues) section.
- Create a new issue with detailed information.
- Refer to the documentation for troubleshooting tips.

---

## 📜 License

This repository is licensed under the Apache License License. See the `LICENSE` file for details.

---

## 🙌 Acknowledgments

- Waveshare for their excellent hardware platforms and software support
- The Espressif Team for their continuous support.
- Open-source contributors who make these projects possible.

---

Thank you for using Waveshare-ESP32-components! 🚀

