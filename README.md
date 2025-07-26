# 🧢 LifeSaver Helmet 🚨

A smart IoT-based helmet system that enhances motorcycle safety by detecting accidents, alcohol consumption, overspeeding, and helmet status. The system sends automatic emergency alerts with GPS location via GSM and features a Bluetooth-connected Android app for real-time monitoring and configuration.

---

## 🛠️ Features

- ⚠️ **Fall Detection**: Detects sudden falls using MPU6050 and sends SOS.
- 🍸 **Alcohol Detection**: Detects if the rider is drunk using MQ3.
- 🧢 **Helmet Wear Detection**: IR sensor checks if the helmet is being worn.
- 🚨 **SOS Button**: Manual SOS trigger with real-time location.
- 📡 **GSM Communication**: Sends emergency messages using SIM800L.
- 📍 **GPS Location Tracking**: Captures accurate coordinates via NEO-6M.
- 🚦 **Overspeed Detection**: Buzzer alert for speeds over 80 km/h.
- 📲 **Bluetooth App**:
  - Update emergency number
  - Monitor helmet status
  - Check module connectivity
  - Send SOS directly from the app

---

## 📦 Components Used

| Component        | Purpose                              |
|------------------|--------------------------------------|
| ESP32 (Helmet)   | Main controller and sensor hub       |
| ESP8266 (Bike)   | Controls bike ignition via ESP-NOW   |
| SIM800L          | Sends SMS alerts                     |
| NEO-6M GPS       | Captures current GPS location        |
| MPU6050          | Detects fall via acceleration/gyro  |
| MQ3 Sensor       | Detects alcohol presence             |
| IR Sensor        | Detects helmet being worn            |
| Relay Module     | Controls bike ignition               |
| Buzzer & LEDs    | Alerts for overspeed and system status |

---

## 🔧 Setup Instructions

1. **Hardware**
   - Assemble the sensors and modules based on the wiring diagram.
   - Connect ESP32 for helmet, ESP8266 for bike.
   - Power the system using 7.4V Li-ion battery or 5V regulator.

2. **Flashing Code**
   - Upload `helmet_module.ino` to ESP32.
   - Upload `bike_module.ino` to NodeMCU (ESP8266).
   - Use Arduino IDE with required libraries:
     - Adafruit_MPU6050
     - TinyGPS++
     - BluetoothSerial
     - Preferences
     - esp_now

3. **App Installation**
   - Import the `.aia` file in [MIT App Inventor](https://appinventor.mit.edu/).
   - Build APK and install on your Android phone.
   - Pair with the helmet via Bluetooth.

4. **Usage**
   - Power on helmet and bike modules.
   - Connect to the app via Bluetooth.
   - Configure emergency number.
   - Monitor status, send SOS, or receive automatic alerts.

---

## 📱 Android App (MIT App Inventor)

- 🛠️ **Functions**:
  - Emergency number configuration
  - Real-time sensor status display
  - SOS trigger from phone
  - GSM fallback using the app if SMS sending fails

- 📲 **How to use**:
  - Install the APK on your phone
  - Pair with the helmet Bluetooth module
  - Navigate the user-friendly interface to access all features


---
## Screenshots
![Homepage](https://raw.githubusercontent.com/jh-emon002/LifeSaverHelmet/master/images/app status.jpg)




---
## 👥 Credits

**Project Name**: LifeSaver Helmet  
**Developed by**:  
- Md. Jaied Hasan  
- Erom Hasan Niha  
- Amir Hamza  
- Ayon Kumar Dey  
**Department of EEE**, Bangladesh University of Engineering and Technology (BUET)

---

## 📜 License

© 2025 All Rights Reserved.  
For academic and research purposes only.

---

## 📎 Related Files

- `helmet_module.ino` - ESP32 code  
- `bike_module.ino` - ESP8266 code  
- `LifeSaverHelmet.aia` - Android App Project
-  `LifeSaverHelmet.apk` - Android App Package file  
- Wiring diagrams and implementation report available in repo
