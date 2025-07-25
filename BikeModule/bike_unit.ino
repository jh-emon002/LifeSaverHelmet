#include <ESP8266WiFi.h>
#include <espnow.h>

// Struct for received data
typedef struct struct_message {
  bool helmetWorn;
  bool alcoholDetected;
  float speed;
} struct_message;

struct_message incomingData;

#define RELAY_PIN D6
#define LED_PIN   D5 
#define SPEED_LIMIT 60.0

unsigned long lastBlinkTime = 0;
bool shouldBlink = false;

void OnDataRecv(uint8_t *mac, uint8_t *incomingDataRaw, uint8_t len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  Serial.println("📥 ESP-NOW Data Received:");
  Serial.print("🧢 Helmet Worn: ");
  Serial.println(incomingData.helmetWorn ? "Yes" : "No");
  Serial.print("🍸 Alcohol Detected: ");
  Serial.println(incomingData.alcoholDetected ? "Yes" : "No");
  Serial.print("🏍️ Speed: ");
  Serial.print(incomingData.speed);
  Serial.println(" km/h");

  if (incomingData.helmetWorn && !incomingData.alcoholDetected) {
    digitalWrite(RELAY_PIN, LOW);
  } else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  // Speed check
  shouldBlink = incomingData.speed > SPEED_LIMIT;
  if (!shouldBlink) {
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);


  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); 
  digitalWrite(LED_PIN, LOW);   


  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); 

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("✅ ESP8266 Receiver Ready (Relay + LED)");
}

void loop() {
  if (shouldBlink) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= 500) {
      lastBlinkTime = now;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
    }
  }
}


