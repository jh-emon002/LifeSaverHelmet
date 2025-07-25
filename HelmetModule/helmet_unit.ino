
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// Modules
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2 for GPS
HardwareSerial sim800(1);     // UART1 for SIM800L
BluetoothSerial SerialBT;     // Bluetooth Serial
Preferences prefs;            // For storing emergency number

// Pins
const int buzzerPin = 4;
const int sosPin = 12;
const int IRPin = 14;
const int alcoholPin = 18;

// Thresholds
const float FALL_ACCEL_DIFF_THRESHOLD = 4;
const float FALL_GYRO_THRESHOLD = 4;
const float SPEED_LIMIT = 80.0;

// State variables
unsigned long lastSpeedCheck = 0;
unsigned long overspeedStart = 0;
bool buzzerOn = false;
bool fallDetected = false;
bool sosSent = false;
float oldAccTotal = 0.0;
String emergencyNumber = "";

typedef struct struct_message {
  bool helmetWorn;
  bool alcoholDetected;
  float speed;
} struct_message;

struct_message outgoingData;

uint8_t esp8266Mac[] = {0x08, 0x3A, 0x8D, 0xDE, 0xCA, 0xBC};



void setup() {
  Serial.begin(115200);
  SerialBT.begin("SmartHelmet");
  Wire.begin(21, 22);

  WiFi.mode(WIFI_STA);

if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW init failed!");
  return;
}

esp_now_peer_info_t peerInfo = {};
memcpy(peerInfo.peer_addr, esp8266Mac, 6);
peerInfo.channel = 0;
peerInfo.encrypt = false;

if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  Serial.println("Failed to add peer");
  return;
}


  pinMode(buzzerPin, OUTPUT);
  pinMode(IRPin, INPUT);
  pinMode(sosPin, INPUT_PULLUP);
  pinMode(alcoholPin, INPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(2, OUTPUT);

  prefs.begin("helmet", false);
  emergencyNumber = prefs.getString("emg_num", "+8801518951511");
  Serial.println("📞 Stored Emergency Number: " + emergencyNumber);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found.");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 initialized.");

  sim800.begin(9600, SERIAL_8N1, 26, 27);
  delay(2000);
  Serial.println("SIM800L initialized.");
  sim800.println("AT+CMGF=1");
  delay(1000);

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS module initialized.");
  Serial.println(" Bluetooth ready. Send emergency number via SerialBT.");
}

void loop() {

if (SerialBT.available()) {
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);

  String cmd = SerialBT.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("+880") && cmd.length() >= 11) {
    emergencyNumber = cmd;
    prefs.putString("emg_num", emergencyNumber);
    SerialBT.println("✅ Emergency number updated to: " + emergencyNumber);
    Serial.println("✅ Emergency number updated to: " + emergencyNumber);
  } 
  else if (cmd.equalsIgnoreCase("status")) {
  sim800.println("AT+CREG?");
  delay(500);

  String gsmResp = "";
  while (sim800.available()) {
    gsmResp += char(sim800.read());
  }

  String gsmStatus;
  if (gsmResp.indexOf("+CREG: 0,1") != -1) {
    gsmStatus = "CONNECTED";
  } else {
    gsmStatus = "DISCONNECTED";
  }

  // --- GPS status ---
  String gpsStatus = gps.location.isValid() ? "OK" : "NOT FIXED";

  // --- MPU6050 status ---
  bool mpuStatus = true; // assumed working (you can use a flag from setup)

  // --- Helmet status ---
  String helmetStatus = digitalRead(IRPin) == LOW ? "WORN" : "NOT WORN";

    // --- Alcohol status ---
  String alcoholStatus = digitalRead(alcoholPin) == LOW ? "DRUNK" : "NOT DRUNK";

  // --- Speed ---
  float speed = gps.speed.kmph();


  String location = getGPSLocation();

  SerialBT.println("📡 STATUS REPORT:\n");
  SerialBT.println("📶 GSM: " + gsmStatus);
  SerialBT.println("📍 GPS: " + gpsStatus);
  SerialBT.println("📈 MPU6050: " + String(mpuStatus ? "OK" : "NOT OK"));
  SerialBT.println("🧢 Helmet: " + helmetStatus);
  SerialBT.println("🍸 Alcohol: " + alcoholStatus);
  SerialBT.println("🏍️ Speed: " + String(speed, 1) + " km/h");
  SerialBT.println("📍 Location: " + String(gps.location.isValid() ? "https://maps.google.com/maps?q=loc:" + location : "GPS NOT FIXED"));
}

  else if (cmd.equalsIgnoreCase("number")) {
  SerialBT.println("📞 Current Emergency Number: " + emergencyNumber);
}


  else if (cmd.equalsIgnoreCase("sos")) {
  SerialBT.println("🚨 SOS Pressed");
  String location = getGPSLocation();
      if (location == "GPS not fixed")
        sendSMS(emergencyNumber, "SOS Pressed. Unable to get location.");
      else
        sendSMS(emergencyNumber, "I'm in danger. Help me! Location: https://maps.google.com/maps?q=loc:" + location);
      sosSent = true;
}

  else {
    SerialBT.println("❌ Invalid number format. Use +880xxxxxxxxxx or tap 'status'");
  }
}


  // Read GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // MPU6050 readings
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  float accTotal = sqrt(accel.acceleration.x * accel.acceleration.x +
                        accel.acceleration.y * accel.acceleration.y +
                        accel.acceleration.z * accel.acceleration.z);
  float gyroTotal = sqrt(gyro.gyro.x * gyro.gyro.x +
                         gyro.gyro.y * gyro.gyro.y +
                         gyro.gyro.z * gyro.gyro.z);
  float diff = abs(accTotal - oldAccTotal);
  oldAccTotal = accTotal;

  Serial.print("ΔAcc: "); Serial.print(diff, 2);
  Serial.print(" m/s^-2 | Gyro: "); Serial.print(gyroTotal, 2);
  Serial.print(" rad/s\n");

  if (diff > FALL_ACCEL_DIFF_THRESHOLD && gyroTotal > FALL_GYRO_THRESHOLD && !fallDetected) {
    fallDetected = true;
    Serial.println("⚠️ Fall detected!");
    SerialBT.println("⚠️ Fall detected!");
    String location = getGPSLocation();
    if (location == "GPS not fixed")
      sendSMS(emergencyNumber, "Fall detected. Unable to get location.");
    else
      sendSMS(emergencyNumber, "Fall detected. Location: https://maps.google.com/maps?q=loc:" + location);
    delay(10000);
  } else if (diff < 0.5 && gyroTotal < 0.5) {
    fallDetected = false;
  }

  // SOS button
  if (digitalRead(sosPin) == LOW && !sosSent) {
    delay(3000);
    if (digitalRead(sosPin) == LOW) {
      SerialBT.println("🚨 SOS Pressed");
      String location = getGPSLocation();
      if (location == "GPS not fixed")
        sendSMS(emergencyNumber, "SOS Pressed. Unable to get location.");
      else
        sendSMS(emergencyNumber, "I'm in danger. Help me! Location: https://maps.google.com/maps?q=loc:" + location);
      sosSent = true;
    }
  } else if (digitalRead(sosPin) == HIGH) {
    sosSent = false;
  }

  checkSpeed();

bool helmetStatus = digitalRead(IRPin) == LOW;
bool alcoholStatus = digitalRead(alcoholPin) == LOW;
float speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

sendHelmetData(helmetStatus, alcoholStatus, speed);

  delay(200);
}

void checkSpeed() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSpeedCheck >= 5000) {
    lastSpeedCheck = currentMillis;

    if (gps.speed.isValid()) {
      float speed = gps.speed.kmph();
      Serial.print("Speed: "); Serial.println(speed);
      if (speed > SPEED_LIMIT) {
        if (overspeedStart == 0) overspeedStart = currentMillis;
        else if (currentMillis - overspeedStart >= 10000 && !buzzerOn) {
          Serial.println("🚨 Overspeed detected! Buzzer ON.");
          SerialBT.println("🚨 Overspeed, Buzzer ON.");
          digitalWrite(buzzerPin, HIGH);
          buzzerOn = true;
        }
      } else {
        overspeedStart = 0;
        if (buzzerOn) {
          digitalWrite(buzzerPin, LOW);
          buzzerOn = false;
          Serial.println("✅ Speed normalized. Buzzer OFF.");
          SerialBT.println("✅ Speed normalized. Buzzer OFF.");
        }
      }
    }
  }
}

String getGPSLocation() {
  if (gps.location.isValid()) {
    return String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  } else {
    return "GPS not fixed";
  }
}

void sendSMS(String number, String message) {
  Serial.println("📤 Sending SMS...");
  SerialBT.println("📤 Sending SMS...");
  sim800.println("AT+CMGF=1");
  delay(500);
  sim800.print("AT+CMGS=\"");
  sim800.print(number);
  sim800.println("\"");
  delay(500);
  sim800.print(message);
  sim800.write(26);
  delay(5000);

  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    while (sim800.available()) {
      response += (char)sim800.read();
    }
  }

  String responseUpper = response;
  responseUpper.toUpperCase();
  Serial.println("📩 SIM800L response:");
  if (responseUpper.indexOf("OK") != -1 && responseUpper.indexOf("+CMGS") != -1){
    Serial.println("✅ SMS sent successfully.");
    SerialBT.println("✅ SMS sent successfully.");}
  else{
    Serial.println("❌ SMS failed to send.");
    SerialBT.println("❌ SMS failed to send.");
}}


void sendHelmetData(bool helmetWorn, bool alcoholDetected, float speed) {
  outgoingData.helmetWorn = helmetWorn;
  outgoingData.alcoholDetected = alcoholDetected;
  outgoingData.speed = speed;

  esp_err_t result = esp_now_send(esp8266Mac, (uint8_t *)&outgoingData, sizeof(outgoingData));

  if (result == ESP_OK) {
    Serial.println("✅ ESP-NOW data sent");
  } else {
    Serial.println("❌ Error sending ESP-NOW data");
  }
}



