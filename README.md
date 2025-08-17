# Fall Detection & Health Monitoring System for Elderly Safety  

An **IoT-based safety system** built using ESP32, MPU6050 (accelerometer + gyroscope), MAX30102 (heart rate & SpO₂ sensor), GPS, and Blynk.  
This project detects falls in elderly individuals and monitors vital signs, sending alerts with location data to caregivers.  

## 🚀 Features  
- 📉 Detects falls using motion sensor (MPU6050).  
- ❤️ Monitors **heart rate** and **SpO₂** using MAX30102 sensor.  
- 📍 Tracks location with GPS (NEO-6M module).  
- 📲 Sends **real-time alerts** via Blynk (push notification).  
- 🔊 Buzzer alert for immediate surroundings.  

## 🛠️ Hardware Components  
- ESP32  
- MPU6050 (Accelerometer + Gyroscope)  
- MAX30102 (Heart Rate & SpO₂ sensor)  
- NEO-6M GPS module  
- Buzzer  

## 🛠️ Software & Tools  
- Arduino IDE  
- Blynk IoT  
- C / C++  

## 📂 Files in this Repository  
- `falldetectioncode.ino` → Arduino code for ESP32.  
- `report.pdf` → Project report / synopsis with details.  
- `circuit_diagram.png` → Circuit connections (if available).  

## ⚡ How to Run  
1. Open `falldetectioncode.ino` in Arduino IDE.  
2. Install required libraries:  
   - `Wire.h`  
   - `Adafruit_MPU6050.h`  
   - `Adafruit_Sensor.h`  
   - `Adafruit_MAX30102.h`  
   - `TinyGPS++`  
   - `Blynk`  
3. Connect hardware components as per circuit.  
4. Upload the code to ESP32.  
5. Open Blynk app to view data and receive alerts.  

## 📄 Report  
For detailed methodology, refer to the [Project Report](report.pdf).  

## 📸 Screenshots / Demo  
(Add your device photos, Blynk screenshots, or demo video links here.)  

## 🎯 Applications  
- Elderly health monitoring  
- Medical emergency detection  
- Wearable IoT devices for healthcare  
