#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <MPU6050.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <math.h>

// ======================
// Pin definitions
// ======================
#define RAIN_PIN    A0
#define SOUND_PIN   A1

#define TRIG_PIN    9
#define ECHO_PIN    8

#define DHT_PIN     2
#define DHT_TYPE    DHT11

#define BUZZER_PIN  6

// ======================
// Sensor objects
// ======================
Adafruit_BMP280 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
MPU6050 mpu;
hd44780_I2Cexp lcd;

// ======================
// Globals
// ======================
bool bmpReady = false;
bool mpuReady = false;
bool lcdReady = false;

float baselinePressure = 0.0;

unsigned long lastLCDPageChange = 0;
int lcdPage = 0;

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

void sendDataOneLine(
  int rainValue,
  int soundValue,
  float temperature,
  float humidity,
  float pressure,
  float vibration,
  float distance,
  int hazardCode
);

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  dht.begin();

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
  }

  // ----------------------
  // MPU6050 setup
  // ----------------------
  mpu.initialize();
  if (mpu.testConnection()) {
    mpuReady = true;
  }

  if (lcdReady) {
    lcd.setCursor(0, 0);
    lcd.print("Setup complete  ");
    lcd.setCursor(0, 1);
    lcd.print("Serial -> Pi    ");
  }

  delay(1500);

  // CSV header for Raspberry Pi / logging
  Serial.println("rain,sound,temp,humidity,pressure,vibration,distance,hazardCode,hazardText");
}

void loop() {
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

  // ----------------------
  // Buzzer for abnormal hazard
  // ----------------------
  if (hazardCode != 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
     delay(300);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ----------------------
  // Send one line to Raspberry Pi
  // ----------------------
  sendDataOneLine(
    rainValue,
    soundValue,
    temperature,
    humidity,
    pressure,
    vibration,
    distance,
    hazardCode
  );

  // ----------------------
  // LCD update
  // ----------------------
  updateLCDStable(
    temperature,
    humidity,
    pressure,
    hazardCode
  );

  delay(1500);
}

// --------------------------------------------------
// Send one CSV line over Serial
// --------------------------------------------------
void sendDataOneLine(
  int rainValue,
  int soundValue,
  float temperature,
  float humidity,
  float pressure,
  float vibration,
  float distance,
  int hazardCode
) {
  Serial.print(rainValue);
  Serial.print(",");

  Serial.print(soundValue);
  Serial.print(",");

  if (isnan(temperature)) Serial.print("nan");
  else Serial.print(temperature, 2);
  Serial.print(",");

  if (isnan(humidity)) Serial.print("nan");
  else Serial.print(humidity, 2);
  Serial.print(",");

  if (pressure < 0) Serial.print("nan");
  else Serial.print(pressure, 2);
  Serial.print(",");

  if (vibration < 0) Serial.print("nan");
  else Serial.print(vibration, 3);
  Serial.print(",");

  if (distance < 0) Serial.print("nan");
  else Serial.print(distance, 2);
  Serial.print(",");

  Serial.print(hazardCode);
  Serial.print(",");

  Serial.println(hazardCodeToText(hazardCode));
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

  // deviation from 1g
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
  bool rainDetected = (rainValue < 500);
  bool loudSound = (soundValue > 600);
  bool highVibration = (vibration > 0.50);
  bool strongVibration = (vibration > 1.00);
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
// Hazard code text
// --------------------------------------------------
const char* hazardCodeToText(int code) {
  switch (code) {
    case 1: return "FLOOD_RISK";
    case 2: return "STORM_WARNING";
    case 3: return "STRUCTURAL_ALERT";
    default: return "NORMAL";
  }
}

// --------------------------------------------------
// LCD updater
// Screen 1: T, H, P
// Screen 2: Hazard status
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
  else {
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