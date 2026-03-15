#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <MPU6050.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <math.h>

#include <WiFiS3.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// ======================
// WiFi / Adafruit IO
// ======================
#define WLAN_SSID       "Hack"
#define WLAN_PASS       "12345678"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "HackstreetBois"
#define AIO_KEY         "aio_VDVy41iV6lueNRfNjq50EkIAazKI"

// ======================
// Pin definitions
// ======================
#define RAIN_PIN   A0
#define SOUND_PIN  A1

#define TRIG_PIN   7
#define ECHO_PIN   6

#define DHT_PIN    2
#define DHT_TYPE   DHT11   // Change to DHT22 if needed
#define BUZZER_PIN 6
// ======================
// Sensor objects
// ======================
Adafruit_BMP280 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
MPU6050 mpu;
hd44780_I2Cexp lcd;

// ======================
// MQTT objects
// ======================
WiFiClient wifiClient;
Adafruit_MQTT_Client mqtt(&wifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feed names must match EXACTLY in Adafruit IO
Adafruit_MQTT_Publish rainFeed         = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rain");
Adafruit_MQTT_Publish soundFeed        = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sound");
Adafruit_MQTT_Publish temperatureFeed  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidityFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish pressureFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish vibrationFeed    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/vibration");
Adafruit_MQTT_Publish waterFeed        = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/water_distance");
Adafruit_MQTT_Publish hazardCodeFeed   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hazard_code");
Adafruit_MQTT_Publish hazardTextFeed   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hazard_text");

// ======================
// Globals
// ======================
bool bmpReady = false;
bool mpuReady = false;
bool lcdReady = false;

float baselinePressure = 0.0;

unsigned long lastLCDPageChange = 0;
int lcdPage = 0;

unsigned long lastPublish = 0;
const unsigned long publishIntervalMs = 40000;   // 40 seconds

// ======================
// Function declarations
// ======================
float readDistanceCM();
float readVibrationLevel();
int classifyHazardCode(
  int rainValue,
  int soundValue,
  float temperature,
  float humidity,
  float pressure,
  float vibration,
  float distance
);
const char* hazardCodeToText(int code);
void updateLCDStable(
  float temperature,
  float humidity,
  float pressure,
  int hazardCode
);
void connectWiFi();
void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("====================================");
  Serial.println(" Smart Environmental Hazard Node");
  Serial.println(" UNO R4 WiFi + LCD + Adafruit IO");
  Serial.println("====================================");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();
  dht.begin();
pinMode(BUZZER_PIN, OUTPUT);
digitalWrite(BUZZER_PIN, LOW);
  // ----------------------
  // LCD setup
  // ----------------------
  int lcdStatus = lcd.begin(16, 2);
  if (lcdStatus == 0) {
    lcdReady = true;
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Hazard Node     ");
    lcd.setCursor(0, 1);
    lcd.print("Initializing... ");
  } else {
    Serial.print("LCD init failed, status = ");
    Serial.println(lcdStatus);
  }

  // ----------------------
  // BMP280 setup
  // ----------------------
  if (bmp.begin(0x76)) {
    bmpReady = true;
  } else if (bmp.begin(0x77)) {
    bmpReady = true;
  }

  if (bmpReady) {
    baselinePressure = bmp.readPressure() / 100.0;
    Serial.println("BMP280 connected.");
    Serial.print("Baseline Pressure (hPa): ");
    Serial.println(baselinePressure);
  } else {
    Serial.println("BMP280 NOT detected.");
  }

  // ----------------------
  // MPU6050 setup
  // ----------------------
  mpu.initialize();
  if (mpu.testConnection()) {
    mpuReady = true;
    Serial.println("MPU6050 connected.");
  } else {
    Serial.println("MPU6050 NOT detected.");
  }

  // ----------------------
  // WiFi setup
  // ----------------------
  connectWiFi();
  MQTT_connect();

  if (lcdReady) {
    lcd.setCursor(0, 0);
    lcd.print("Setup complete  ");
    lcd.setCursor(0, 1);
    lcd.print("WiFi connected  ");
  }

  delay(1500);
}

void loop() {
  // ----------------------
  // Reconnect if needed
  // ----------------------
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  MQTT_connect();

  // ----------------------
  // Read sensors
  // ----------------------
  int rainValue = analogRead(RAIN_PIN);
  int soundValue = analogRead(SOUND_PIN);

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  float pressure = -1.0;
  if (bmpReady) {
    pressure = bmp.readPressure() / 100.0; // hPa
  }

  float vibration = -1.0;
  if (mpuReady) {
    vibration = readVibrationLevel();
  }

  float distance = readDistanceCM();

  int hazardCode = classifyHazardCode(
    rainValue,
    soundValue,
    temperature,
    humidity,
    pressure,
    vibration,
    distance
  );
if (hazardCode != 0) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
} else {
  digitalWrite(BUZZER_PIN, LOW);
}
  // ----------------------
  // Serial output
  // ----------------------
  Serial.println("----------- SENSOR READINGS -----------");

  Serial.print("Rain Sensor: ");
  Serial.println(rainValue);

  Serial.print("Sound Sensor: ");
  Serial.println(soundValue);

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT Sensor: Failed reading");
  } else {
    Serial.print("Temperature (C): ");
    Serial.println(temperature);

    Serial.print("Humidity (%): ");
    Serial.println(humidity);
  }

  if (bmpReady) {
    Serial.print("Pressure (hPa): ");
    Serial.println(pressure);

    Serial.print("Pressure Change from Baseline (hPa): ");
    Serial.println(pressure - baselinePressure);
  } else {
    Serial.println("BMP280: Not available");
  }

  if (mpuReady) {
    Serial.print("Vibration Level: ");
    Serial.println(vibration);
  } else {
    Serial.println("MPU6050: Not available");
  }

  Serial.print("Ultrasonic Distance (cm): ");
  Serial.println(distance);

  Serial.print("Hazard Status: ");
  Serial.println(hazardCodeToText(hazardCode));

  Serial.println("---------------------------------------");
  Serial.println();

  // ----------------------
  // LCD update
  // ----------------------
  updateLCDStable(
    temperature,
    humidity,
    pressure,
    hazardCode
  );

  // ----------------------
  // Publish to Adafruit IO
  // ----------------------
  if (millis() - lastPublish >= publishIntervalMs) {
    lastPublish = millis();

    bool ok = true;

    ok &= rainFeed.publish((int32_t)rainValue);
    ok &= soundFeed.publish((int32_t)soundValue);

    if (!isnan(temperature)) ok &= temperatureFeed.publish((double)temperature);
    if (!isnan(humidity))    ok &= humidityFeed.publish((double)humidity);
    if (pressure >= 0)       ok &= pressureFeed.publish((double)pressure);
    if (vibration >= 0)      ok &= vibrationFeed.publish((double)vibration);
    if (distance >= 0)       ok &= waterFeed.publish((double)distance);

    ok &= hazardCodeFeed.publish((int32_t)hazardCode);
    ok &= hazardTextFeed.publish(hazardCodeToText(hazardCode));

    if (ok) {
      Serial.println("Published all sensor values to Adafruit IO.");
    } else {
      Serial.println("One or more publishes failed.");
    }

    mqtt.ping();
  }

  delay(1500);
}

// --------------------------------------------------
// WiFi connect
// --------------------------------------------------
void connectWiFi() {
  Serial.print("Connecting to WiFi");

  while (WiFi.begin(WLAN_SSID, WLAN_PASS) != WL_CONNECTED) {
    Serial.print(".");
    delay(3000);
  }

  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// --------------------------------------------------
// MQTT connect
// --------------------------------------------------
void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to Adafruit IO MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT in 5 seconds...");
    mqtt.disconnect();
    delay(5000);

    if (--retries == 0) {
      Serial.println("MQTT connection failed.");
      return;
    }
  }

  Serial.println("MQTT Connected!");
}

// --------------------------------------------------
// Ultrasonic sensor distance
// --------------------------------------------------
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}

// --------------------------------------------------
// MPU6050 vibration estimate
// --------------------------------------------------
float readVibrationLevel() {
  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  return fabs(magnitude - 1.0);
}

// --------------------------------------------------
// Hazard classification
// 0 = NORMAL
// 1 = FLOOD
// 2 = STORM
// 3 = STRUCTURAL
// --------------------------------------------------
int classifyHazardCode(
  int rainValue,
  int soundValue,
  float temperature,
  float humidity,
  float pressure,
  float vibration,
  float distance
) {
  bool rainDetected = (rainValue < 500);        // tune after testing
  bool loudSound = (soundValue > 600);          // tune after testing
  bool highVibration = (vibration > 0.50);      // tune after testing
  bool strongVibration = (vibration > 1.00);    // tune after testing
  bool waterRising = (distance > 0 && distance < 15);

  bool stormRisk = false;
  if (!isnan(humidity) && pressure > 0) {
    if (humidity > 75 && pressure < (baselinePressure - 3.0)) {
      stormRisk = true;
    }
  }

  if (rainDetected && waterRising) {
    return 1; // FLOOD
  }

  if (stormRisk) {
    return 2; // STORM
  }

  if (strongVibration || (highVibration && loudSound)) {
    return 3; // STRUCTURAL
  }

  return 0; // NORMAL
}

// --------------------------------------------------
// Converts hazard code to text
// --------------------------------------------------
const char* hazardCodeToText(int code) {
  switch (code) {
    case 1: return "FLOOD RISK";
    case 2: return "STORM WARNING";
    case 3: return "STRUCTURAL ALERT";
    default: return "NORMAL";
  }
}

// --------------------------------------------------
// LCD updater
// Screen 1: temperature, humidity and pressure
// Screen 2: hazard status on two lines
// --------------------------------------------------
void updateLCDStable(
  float temperature,
  float humidity,
  float pressure,
  int hazardCode
) {
  if (!lcdReady) return;

  if (millis() - lastLCDPageChange > 3000) {
    lcdPage = (lcdPage + 1) % 2;
    lastLCDPageChange = millis();
  }

  if (lcdPage == 0) {
    lcd.setCursor(0, 0);
    lcd.print("T:");
    if (isnan(temperature)) {
      lcd.print("Err ");
    } else {
      lcd.print(temperature, 1);
      lcd.print("C ");
    }

    lcd.print("H:");
    if (isnan(humidity)) {
      lcd.print("Err ");
    } else {
      lcd.print(humidity, 0);
      lcd.print("% ");
    }

    lcd.print("   ");

    lcd.setCursor(0, 1);
    lcd.print("P:");
    if (pressure < 0) {
      lcd.print("Err            ");
    } else {
      lcd.print(pressure, 1);
      lcd.print(" hPa       ");
    }
  }
  else if (lcdPage == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Hazard Status:  ");

    lcd.setCursor(0, 1);
    if (hazardCode == 0) {
      lcd.print("NORMAL          ");
    }
    else if (hazardCode == 1) {
      lcd.print("FLOOD RISK      ");
    }
    else if (hazardCode == 2) {
      lcd.print("STORM WARNING   ");
    }
    else if (hazardCode == 3) {
      lcd.print("STRUCTURAL ALRT ");
    }
    else {
      lcd.print("UNKNOWN         ");
    }
  }
}