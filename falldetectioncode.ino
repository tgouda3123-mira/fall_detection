#define BLYNK_TEMPLATE_ID "BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Fall Detection"
#define BLYNK_AUTH_TOKEN "BLYNK_AUTH_TOKEN"

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <MPU6050_tockn.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "MAX30105.h"
#include "heartRate.h"

// WiFi credentials
char ssid[] = "wifiname";
char pass[] = "wifipassword";

// Pin Definitions
#define BUZZER_PIN 25
#define GPS_RX 16
#define GPS_TX 17

// Objects
MPU6050 mpu6050(Wire);
BlynkTimer timer;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
MAX30105 particleSensor;

// Globals
bool fallDetected = false;
bool abnormalHeartRateDetected = false;
unsigned long fallTime = 0;
float lastLat = 0.0, lastLon = 0.0;
float beatsPerMinute;
int beatAvg;
unsigned long lastBeat = 0;

// --- Buzzer function ---
void buzz() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(3000);
  digitalWrite(BUZZER_PIN, LOW);
}

// --- GPS Update Function ---
void updateGPS() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
  }
}

// --- Heart Rate Detection Function ---
void checkHeartRate() {
  long irValue = particleSensor.getIR();
  Serial.print("Heart IR: ");
  Serial.println(irValue);

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      beatAvg = (beatAvg * 3 + beatsPerMinute) / 4;

      if (beatAvg > 100) {
        abnormalHeartRateDetected = true;
      } else {
        abnormalHeartRateDetected = false;
      }
    }
  }

  Serial.print("BPM: ");
  Serial.println(beatsPerMinute);
}

// --- Sensor Update and Fall Detection ---
void sendSensorData() {
  mpu6050.update();

  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();
  float gx = mpu6050.getGyroX();
  float gy = mpu6050.getGyroY();
  float gz = mpu6050.getGyroZ();
  float totalAccel = sqrt(ax * ax + ay * ay + az * az);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);
  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" Y: "); Serial.print(gy);
  Serial.print(" Z: "); Serial.println(gz);
  Serial.print("Total Accel: "); Serial.println(totalAccel);

  Blynk.virtualWrite(V0, ax);
  Blynk.virtualWrite(V1, ay);
  Blynk.virtualWrite(V2, az);
  Blynk.virtualWrite(V3, gx);
  Blynk.virtualWrite(V4, gy);
  Blynk.virtualWrite(V5, gz);

  if ((az < 0.3 || ax > 2.0 || ay > 2.0 || totalAccel > 18) &&
      (abs(gx) > 2 || abs(gy) > 2 || abs(gz) > 2)) {

    if (!fallDetected) {
      fallDetected = true;
      fallTime = millis();

      String locationLink = (lastLat != 0.0) ? "https://maps.google.com/?q=" + String(lastLat, 6) + "," + String(lastLon, 6) : "Location not available";
      String alert = "⚠ Fall Detected! Please check.\n" + locationLink;

      Serial.println(alert);
      Blynk.logEvent("fall_alert", alert);

      buzz();
    }
  }

  if (fallDetected && millis() - fallTime > 5000) {
    fallDetected = false;
  }

  checkHeartRate();

  if (abnormalHeartRateDetected) {
    String alert = "⚠ Abnormal Heart Rate Detected! Please check.\nBPM: " + String(beatAvg);
    Serial.println(alert);
    Blynk.logEvent("heart_rate_alert", alert);
    buzz();
  }

  if (fallDetected && abnormalHeartRateDetected) {
    String alert = "⚠ Fall and Abnormal Heart Rate Detected! Please check.\nLocation: " + (lastLat != 0.0 ? "https://maps.google.com/?q=" + String(lastLat, 6) + "," + String(lastLon, 6) : "Location not available");
    Serial.println(alert);
    Blynk.logEvent("fall_and_heart_rate_alert", alert);
    buzz();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(100000);  // Set I2C speed to 100kHz
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println("Initializing MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("✅ MPU6050 Initialized");

  Serial.println("Connecting to WiFi...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("✅ WiFi Connected");

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✅ GPS Initialized");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  Serial.println("✅ Heart Rate Sensor Initialized");

  timer.setInterval(1000L, sendSensorData);
  timer.setInterval(2000L, updateGPS);
}

void loop() {
  Blynk.run();
  timer.run();
}
