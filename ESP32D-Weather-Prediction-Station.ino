/************************************************************
   ESP32 38-pin Weather Prediction Station (BMP280 version)
   Sensors: DHT11, BMP280, MQ135
   Display: 0.9" SSD1306 OLED, SPI (U8g2)

   Pages (auto change every 1 second):
     0 = Weather prediction / summary
     1 = DHT11 (T/H) with bitmap icons & bars
     2 = BMP280 (T/P/Alt) with bitmap icons
     3 = MQ135 air quality bitmap + bar
 ************************************************************/

#include <Arduino.h>

// ---------- DHT11 ----------
#include <DHT.h>
#define DHTPIN  14          // DHT11 data pin
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---------- BMP280 ----------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>   // Adafruit BMP280 library [web:92]
Adafruit_BMP280 bmp;
#define SEALEVELPRESSURE_HPA 1013.25f   // adjust to your local value

// ---------- MQ135 ----------
#define MQ135_PIN 34        // ADC pin for MQ135 A0 (GPIO34 is input only)

// ---------- OLED (U8g2, SPI) ----------
#include <U8g2lib.h>

// SPI pins (software SPI) - change if needed
#define OLED_CLK  16
#define OLED_MOSI 17
#define OLED_CS   18
#define OLED_DC   19
#define OLED_RST   4

// 128x64 SSD1306, full buffer, 4-wire SW SPI
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(
  U8G2_R0,
  /* clock=*/ OLED_CLK,
  /* data=*/  OLED_MOSI,
  /* cs=*/    OLED_CS,
  /* dc=*/    OLED_DC,
  /* reset=*/ OLED_RST
);

// ---------- 16x16 XBM bitmaps ----------
const unsigned char thermometer_16x16_bits[] PROGMEM = {
  0x00,0x00, 0x18,0x00, 0x24,0x00, 0x24,0x00, 0x24,0x00, 0x24,0x00,
  0x24,0x00, 0x24,0x00, 0x24,0x00, 0x24,0x00, 0x24,0x00, 0x3C,0x00,
  0x3C,0x00, 0x3C,0x00, 0x18,0x00, 0x00,0x00
};

const unsigned char humidity_16x16_bits[] PROGMEM = {
  0x00,0x00, 0x08,0x00, 0x18,0x00, 0x28,0x00, 0x48,0x00, 0x88,0x00,
  0x88,0x00, 0x88,0x00, 0x88,0x00, 0x48,0x01, 0x78,0x01, 0x30,0x01,
  0x30,0x01, 0x18,0x00, 0x08,0x00, 0x00,0x00
};

const unsigned char pressure_16x16_bits[] PROGMEM = {
  0x00,0x00, 0x7E,0x00, 0x81,0x01, 0x81,0x01, 0x81,0x01, 0x81,0x01,
  0x81,0x01, 0x81,0x01, 0x81,0x01, 0x81,0x01, 0x81,0x01, 0x81,0x01,
  0x81,0x01, 0x7E,0x00, 0x00,0x00, 0x00,0x00
};

const unsigned char altitude_16x16_bits[] PROGMEM = {
  0x00,0x00, 0x00,0x04, 0x00,0x0E, 0x00,0x1F, 0x00,0x3F, 0x00,0x7F,
  0x00,0xFF, 0x01,0xFF, 0x03,0xFF, 0x07,0xFF, 0x0F,0xFF, 0x1F,0xFF,
  0x3F,0xFF, 0x7F,0xFF, 0xFF,0xFF, 0x00,0x00
};

const unsigned char air_16x16_bits[] PROGMEM = {
  0x00,0x00, 0xFF,0x00, 0x00,0x00, 0x3F,0x00, 0x00,0x00, 0xFF,0x00,
  0x00,0x00, 0x3F,0x00, 0x00,0x00, 0xFF,0x00, 0x00,0x00, 0x3F,0x00,
  0x00,0x00, 0xFF,0x00, 0x00,0x00, 0x00,0x00
};

const unsigned char weather_32x32_bits[] PROGMEM = {
  0x00,0x00,0x00,0x00, 0x00,0xE0,0x07,0x00, 0x00,0xF0,0x0F,0x00,
  0x00,0xF8,0x1F,0x00, 0x00,0xFC,0x3F,0x00, 0x0E,0xFE,0x7F,0x00,
  0x1F,0xFF,0xFF,0x00, 0x3F,0xFF,0xFF,0x00, 0x3F,0xFF,0xFF,0x00,
  0x7F,0xFF,0xFF,0x00, 0x7F,0xFF,0xFF,0x00, 0x3F,0xFF,0xFF,0x00,
  0x3F,0xFF,0xFF,0x00, 0x1F,0xFF,0xFF,0x00, 0x0F,0xFE,0x7F,0x00,
  0x07,0xFC,0x3F,0x00, 0x03,0xF8,0x1F,0x00, 0x01,0xF0,0x0F,0x00,
  0x00,0xE0,0x07,0x00, 0x00,0xC0,0x03,0x00, 0x00,0x80,0x01,0x00,
  0x00,0x00,0x00,0x00, 0x06,0x00,0x60,0x00, 0x0F,0x00,0xF0,0x00,
  0x06,0x00,0x60,0x00, 0x00,0x00,0x00,0x00, 0x18,0x00,0x18,0x00,
  0x3C,0x00,0x3C,0x00, 0x18,0x00,0x18,0x00, 0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00
};

// ---------- globals ----------
uint8_t currentPage = 0;
unsigned long lastSampleMs = 0;
const unsigned long sampleIntervalMs = 2000;
unsigned long lastPageChangeMs = 0;
const unsigned long pageIntervalMs = 5000;  // 5 second

// Sensor readings
float dhtTemp = NAN, dhtHum = NAN;
float bmpTemp = NAN, bmpPress = NAN, bmpAlt = NAN;
int   mq135Raw = 0;
float mq135Pct = 0;

// Pressure trend for forecast
float lastPressure = NAN;
float pressureTrend = 0;
unsigned long lastTrendMs = 0;
const unsigned long trendIntervalMs = 600000UL; // 10 min

// ---------- read sensors ----------
void readSensors() {
  // DHT11
  dhtTemp = dht.readTemperature();
  dhtHum  = dht.readHumidity();

  // BMP280 (no humidity)
  bmpTemp  = bmp.readTemperature();           // °C
  bmpPress = bmp.readPressure() / 100.0f;     // hPa
  bmpAlt   = bmp.readAltitude(SEALEVELPRESSURE_HPA); // m

  // MQ135
  mq135Raw = analogRead(MQ135_PIN);
  mq135Pct = constrain((mq135Raw / 4095.0f) * 100.0f, 0.0f, 100.0f);
}

// ---------- simple forecast ----------
String getForecast(float pressure, float trend) {
  if (isnan(pressure)) return "No data";
  if (pressure > 1020 && trend >= 0.5)  return "Sunny";
  if (pressure > 1010 && trend >  -0.5) return "Fair";
  if (pressure > 1000 && trend <= -0.5) return "Cloudy";
  if (pressure <= 1000 && trend <= -1)  return "Rain?";
  return "Unstable";
}

// ---------- pages ----------

void drawPageWeather() {
  u8g2.clearBuffer();
  u8g2.setBitmapMode(false);
  u8g2.setDrawColor(1);

  // Big weather icon
  u8g2.drawXBM(0, 0, 32, 32, weather_32x32_bits);

  // Forecast
  u8g2.setFont(u8g2_font_6x12_tr);
  String forecast = getForecast(bmpPress, pressureTrend);
  u8g2.drawStr(36, 12, "Forecast:");
  u8g2.drawStr(36, 26, forecast.c_str());

  u8g2.setFont(u8g2_font_5x8_tr);
  char buf[32];
  snprintf(buf, sizeof(buf), "DHT: %.1fC  %.0f%%", dhtTemp, dhtHum);
  u8g2.drawStr(0, 40, buf);

  snprintf(buf, sizeof(buf), "BMP: %.0fhPa Alt:%.0fm", bmpPress, bmpAlt);
  u8g2.drawStr(0, 50, buf);

  snprintf(buf, sizeof(buf), "Air: %3d (%.0f%%)", mq135Raw, mq135Pct);
  u8g2.drawStr(0, 60, buf);

  u8g2.sendBuffer();
}

void drawPageDHT() {
  u8g2.clearBuffer();
  u8g2.setBitmapMode(false);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "DHT11 Temp/Humidity");

  char buf[32];

  // Temperature
  u8g2.drawXBM(0, 16, 16, 16, thermometer_16x16_bits);
  snprintf(buf, sizeof(buf), "T: %.1f C", dhtTemp);
  u8g2.drawStr(20, 28, buf);

  float tNorm = constrain((dhtTemp + 10.0f) * (100.0f / 60.0f), 0.0f, 100.0f);
  uint8_t barLenT = (uint8_t)map((int)tNorm, 0, 100, 0, 100);
  u8g2.drawFrame(20, 32, 100, 6);
  u8g2.drawBox(20, 32, barLenT, 6);

  // Humidity
  u8g2.drawXBM(0, 40, 16, 16, humidity_16x16_bits);
  snprintf(buf, sizeof(buf), "H: %.0f %%", dhtHum);
  u8g2.drawStr(20, 52, buf);

  uint8_t barLenH = (uint8_t)map((int)dhtHum, 0, 100, 0, 100);
  u8g2.drawFrame(20, 56, 100, 6);
  u8g2.drawBox(20, 56, barLenH, 6);

  u8g2.sendBuffer();
}

void drawPageBMP() {
  u8g2.clearBuffer();
  u8g2.setBitmapMode(false);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "BMP280 T/P/Alt");

  char buf[32];

  // Temperature
  u8g2.drawXBM(0, 12, 16, 16, thermometer_16x16_bits);
  snprintf(buf, sizeof(buf), "T: %.1f C", bmpTemp);
  u8g2.drawStr(20, 22, buf);

  // Pressure
  u8g2.drawXBM(0, 30, 16, 16, pressure_16x16_bits);
  snprintf(buf, sizeof(buf), "P: %.0f hPa", bmpPress);
  u8g2.drawStr(20, 40, buf);

  // Altitude
  u8g2.drawXBM(0, 48, 16, 16, altitude_16x16_bits);
  snprintf(buf, sizeof(buf), "Alt: %.0f m", bmpAlt);
  u8g2.drawStr(20, 58, buf);

  u8g2.sendBuffer();
}

void drawPageMQ135() {
  u8g2.clearBuffer();
  u8g2.setBitmapMode(false);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "MQ135 Air Quality");

  u8g2.drawXBM(0, 18, 16, 16, air_16x16_bits);

  char buf[32];
  snprintf(buf, sizeof(buf), "Raw: %4d", mq135Raw);
  u8g2.drawStr(20, 24, buf);

  snprintf(buf, sizeof(buf), "Level: %.0f %%", mq135Pct);
  u8g2.drawStr(20, 36, buf);

  const char *qual = "Good";
  if (mq135Pct > 30) qual = "Moderate";
  if (mq135Pct > 60) qual = "Bad";
  if (mq135Pct > 80) qual = "Very Bad";
  snprintf(buf, sizeof(buf), "Quality: %s", qual);
  u8g2.drawStr(0, 50, buf);

  uint8_t barLen = (uint8_t)map((int)mq135Pct, 0, 100, 0, 120);
  u8g2.drawFrame(4, 54, 120, 8);
  u8g2.drawBox(4, 54, barLen, 8);

  u8g2.sendBuffer();
}

// ---------- page dispatcher ----------
void drawCurrentPage() {
  switch (currentPage) {
    case 0: drawPageWeather(); break;
    case 1: drawPageDHT();     break;
    case 2: drawPageBMP();     break;
    case 3: drawPageMQ135();   break;
    default: currentPage = 0;  break;
  }
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(MQ135_PIN, INPUT);

  // DHT11
  dht.begin();

  // BMP280 I2C (GPIO21=SDA, GPIO22=SCL)
  Wire.begin(21, 22);
  if (!bmp.begin(0x76)) {  // Try 0x76 first, change to 0x77 if needed
    Serial.println("BMP280 not found at 0x76, trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("Could not find BMP280 sensor!");
    } else {
      Serial.println("BMP280 found at 0x77");
    }
  } else {
    Serial.println("BMP280 found at 0x76");
  }

  // OLED
  u8g2.begin();
  u8g2.setContrast(255);

  readSensors();
  lastSampleMs = millis();
  lastTrendMs = millis();
  lastPageChangeMs = millis();
}

// ---------- loop ----------
void loop() {
  unsigned long now = millis();

  // Sample sensors every 2 seconds
  if (now - lastSampleMs >= sampleIntervalMs) {
    lastSampleMs = now;
    readSensors();
  }

  // Pressure trend every 10 minutes
  if (now - lastTrendMs >= trendIntervalMs) {
    lastTrendMs = now;
    if (!isnan(bmpPress)) {
      if (!isnan(lastPressure)) {
        pressureTrend = bmpPress - lastPressure;
      }
      lastPressure = bmpPress;
    }
  }

  // Auto page change every 1 second
  if (now - lastPageChangeMs >= pageIntervalMs) {
    lastPageChangeMs = now;
    currentPage = (currentPage + 1) % 4;
  }

  drawCurrentPage();
}
