# ESP32 Weather Prediction Station

Compact ESP32-based weather station monitoring temperature, humidity, pressure, altitude, and air quality with a cycling OLED display and basic weather forecasting.[1][2]

## Features

- **Multi-Sensor Integration**: DHT11 (temp/humidity), BMP280 (temp/pressure/altitude), MQ135 (air quality).[3][2][1]
- **0.9" SSD1306 OLED Display**: Auto-cycles through 4 intuitive pages every 5 seconds using U8g2 library.[4]
- **Visual Design**: Custom 16x16 bitmap icons, progress bars, and a 32x32 weather icon for engaging UI.
- **Weather Prediction**: Simple algorithm based on pressure trends (e.g., "Sunny", "Rain?").[5]
- **Efficient Sampling**: Sensors read every 2s, pressure trend every 10min.

## Hardware Requirements

- ESP32 (38-pin development board).[6]
- DHT11 sensor.
- BMP280 sensor.
- MQ135 gas sensor.
- 0.9" SSD1306 OLED (128x64, SPI).
- Jumper wires, breadboard.

## Wiring Diagram

| Component | Pin | ESP32 GPIO |
|-----------|-----|------------|
| **DHT11** | VCC | 3.3V |
| | GND | GND |
| | Data | 14 [1][7] |
| **BMP280** | VCC | 3.3V |
| | GND | GND |
| | SDA | 21 (default I2C) [6][8] |
| | SCL | 22 (default I2C) [6][8] |
| **MQ135** | VCC | 5V (or 3.3V) |
| | GND | GND |
| | A0 | 34 (ADC1_CH6, input-only) [9] |
| **SSD1306 OLED (SW SPI)** | VCC | 3.3V |
| | GND | GND |
| | CLK (SCK) | 16 [10] |
| | MOSI (SDA) | 17 [10] |
| | CS | 18 |
| | DC | 19 |
| | RST | 4 [4][10] |

**Notes**: BMP280 I2C addresses tried: 0x76 (primary), 0x77 (fallback). Use voltage divider for MQ135 if 5V output exceeds 3.3V ADC max.[11][12][13]

## Software Setup

1. Install **Arduino IDE** and add ESP32 board support (Boards Manager: "esp32").
2. Install libraries via Library Manager:
   - `DHT sensor library` by Adafruit[1]
   - `Adafruit BMP280 Library` by Adafruit[2][14]
   - `U8g2` by olikraus[15][4]
3. Open `main.ino`, select **ESP32 Dev Module**, choose correct port, upload.
4. Open Serial Monitor (115200 baud) for BMP280 detection logs.

## Display Pages

- **Page 0 (Summary)**: Weather forecast icon + all sensor summary.
- **Page 1 (DHT11)**: Temp/humidity icons + bar graphs.
- **Page 2 (BMP280)**: Temp/pressure/altitude icons.
- **Page 3 (MQ135)**: Air quality level ("Good" to "Very Bad") + bar.

Pages cycle every 5s; full refresh maintains smooth visuals.

## Customization

- **Forecast Logic**: Edit `getForecast()` thresholds in code.
- **Page Timing**: Adjust `pageIntervalMs` (5s), `sampleIntervalMs` (2s).
- **Icons/Bars**: Modify XBM bitmaps or bar mapping functions.
- **Sea Level Pressure**: Update `SEALEVELPRESSURE_HPA` for accurate altitude.[2]
- **MQ135 Calibration**: Raw ADC to % is basic; add calibration for PPM (e.g., CO2).[16][17]

## Troubleshooting

- **No BMP280**: Check I2C wiring (21/22), try address 0x77.[12][13]
- **OLED Blank**: Verify SW SPI pins (16/17/18/19/4), `u8g2.begin()`.[10]
- **NaN Readings**: Sensor power/wiring; DHT11 needs 10kΩ pull-up if unstable.
- **ADC Maxed**: MQ135 raw >4095? Use voltage divider.[9][11]

## Future Enhancements

- WiFi data upload (Blynk/ThingSpeak).
- Deep sleep for battery power.
- Advanced forecast (e.g., humidity + trend).
- Touch buttons for manual page navigation.

Star/fork if useful! Contributions welcome via PRs.[3][5]

[1](https://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-sensor-arduino-ide/)
[2](https://github.com/adafruit/Adafruit_BMP280_Library)
[3](https://github.com/maaz-siddiqui/ESP32_MQ-135_SH1106)
[4](https://nodemcu.readthedocs.io/en/dev-esp32/modules/u8g2/)
[5](https://projecthub.arduino.cc/ballieducation/smart-weather-station-with-esp8266-bmp180-dht11-and-mq135-sensors-local-sd-card-storage-and-web-based-monitoring-cfe75d)
[6](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
[7](https://gndtovcc.home.blog/2020/04/18/micropython-esp32-esp8266-with-dht11-dht22-temperature-and-humidity-sensor/)
[8](https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/)
[9](https://wokwi.com/projects/443872273108956161)
[10](https://github.com/olikraus/u8g2/issues/377)
[11](https://github.com/Bobbo117/MQ135-Air-Quality-Sensor)
[12](https://esphome.io/components/sensor/bmp280/)
[13](https://forum.arduino.cc/t/cant-read-bmp280-sensor-data-using-esp32/1057770)
[14](https://www.arduinolibraries.info/libraries/adafruit-bmp280-library)
[15](https://jensd.dk/doc/esp32/mono-oleds.html)
[16](https://community.home-assistant.io/t/mq135-co-sensor-with-auto-calibrate-feature/963215)
[17](https://help.sinric.pro/pages/tutorials/air-quality-sensors/mq135.html)
[18](https://www.electronicwings.com/esp32/dht11-sensor-interfacing-with-esp32)
[19](https://esp32io.com/tutorials/esp32-dht11-oled)
[20](https://www.youtube.com/watch?v=yQxk5pGRUYg)
[21](https://github.com/adafruit/Adafruit_CircuitPython_BMP280)
[22](https://www.scribd.com/presentation/889845711/IoT-Weather-Station-With-ESP32-1)
[23](https://justdoelectronics.com/esp32-dht11-temperature-and-humidity-sensor/)
[24](https://www.oceanlabz.in/project-3-oled-display-with-esp32-displaying-sensor-data/)
[25](https://robocraze.com/blogs/post/building-a-weather-station-using-esp32-webserver)
[26](https://www.hackster.io/Techatronic/dht11-sensor-with-ssd1306-oled-dht11-with-arduino-ea4ad8)
[27](https://www.reddit.com/r/diyelectronics/comments/112dx6n/esp32_38_pin_pinout_cheat_sheet/)
[28](https://lastminuteengineers.com/esp32-pinout-reference/)
[29](https://www.studiopieters.nl/esp32-pinout/)
[30](https://www.electronicwings.com/esp32/gpio-of-esp32)
[31](https://www.theengineeringprojects.com/2020/12/esp32-pinout-datasheet-features-applications.html)
[32](https://rntlab.com/question/esp32-default-i2c-pins-and-wire-begin/)
[33](https://forum.arduino.cc/t/esp32-u8g2-and-sd1306/688496)
[34](https://github.com/orgs/micropython/discussions/11431)
[35](https://www.sunfounder.com/blogs/news/esp32-pinout-guide-gpio-adc-dac-touch-complete-hardware-reference-for-stable-circuit-design)
[36](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/io_mux.html)
[37](https://github.com/olikraus/u8g2/discussions/1647)
[38](https://community.blynk.cc/t/using-bme280-not-bmp-with-blynk/21431)
