#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <MPU6050.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <math.h>

// ----------------------
// Pin definitions
// ----------------------
#define RAIN_PIN   A0
#define SOUND_PIN  A1

#define TRIG_PIN   9
#define ECHO_PIN   8

#define DHT_PIN    2
#define DHT_TYPE   DHT11   // Change to DHT22 if needed
#define BUZZER_PIN 6
// ----------------------
// Sensor objects
// ----------------------
Adafruit_BMP280 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
MPU6050 mpu;
hd44780_I2Cexp lcd;

// ----------------------
// Globals
// ----------------------
bool bmpReady = false;
bool mpuReady = false;
bool lcdReady = false;

float baselinePressure = 0.0;

unsigned long lastLCDPageChange = 0;
int lcdPage = 0;

// ----------------------
// Function declarations
// ----------------------
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
  int rainValue,
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

  Serial.println("====================================");
  Serial.println(" Smart Environmental Hazard Node");
  Serial.println(" UNO R4 WiFi + LCD + Serial");
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

  if (lcdReady) {
    lcd.setCursor(0, 0);
    lcd.print("Setup complete  ");
    lcd.setCursor(0, 1);
    lcd.print("Starting...     ");
  }

  delay(1500);
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
if (hazardCode != 0) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(300);
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
    rainValue,
    temperature,
    humidity,
    pressure,
    vibration,
    distance,
    hazardCode
  );

  delay(1500);
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

  // Deviation from normal 1g
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
// Convert hazard code to text
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
// Stable LCD updater
// --------------------------------------------------
void updateLCDStable(
  int rainValue,
  float temperature,
  float humidity,
  float pressure,
  float vibration,
  float distance,
  int hazardCode
) {
  if (!lcdReady) return;

  if (millis() - lastLCDPageChange > 3000) {
    lcdPage = (lcdPage + 1) % 2;   // only 2 screens now
    lastLCDPageChange = millis();
  }

  // ----------------------
  // Screen 1: Temp, Humidity, Pressure
  // ----------------------
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

    lcd.print("   ");  // clear leftover chars

    lcd.setCursor(0, 1);
    lcd.print("P:");
    if (pressure < 0) {
      lcd.print("Err            ");
    } else {
      lcd.print(pressure, 1);
      lcd.print(" hPa       ");
    }
  }

  // ----------------------
  // Screen 2: Hazard status on two lines
  // ----------------------
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