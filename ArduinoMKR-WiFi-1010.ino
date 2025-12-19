#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <Servo.h>

// Servo configuration
#define SERVO_PIN 9
#define GATE_OPEN_ANGLE 90
#define GATE_CLOSED_ANGLE 0
#define MAX_CAPACITY 5

// ⚠️ CHANGE THESE TO YOUR DETAILS ⚠️
const char* ssid = "Munna";
const char* password = "01400012301";

// HiveMQ Cloud connection details
const char* mqtt_server = "b78e6b1625884b0c833857b33d446517.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "parking_user";
const char* mqtt_password = "Parking123!";

// MQTT Topics
const char* topic_status = "parking/status";
const char* topic_car_count = "parking/car_count";
const char* topic_gate = "parking/gate";

// Global variables
WiFiSSLClient wifiClient;
MqttClient mqttClient(wifiClient);
Servo gateServo;
int carCounter = 0;
bool gateIsOpen = false;
unsigned long lastCarDetectionTime = 0;
unsigned long lastPublishTime = 0;
unsigned long gateOpenDuration = 5000;
unsigned long publishInterval = 2000; // Publish every 2 seconds

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  
  while (!Serial);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║    SMART PARKING SYSTEM - MQTT         ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Initialize servo
  gateServo.attach(SERVO_PIN);
  gateServo.write(GATE_CLOSED_ANGLE);
  
  // Connect to WiFi
  connectToWiFi();
  
  // Connect to MQTT
  connectToMQTT();
  
  Serial.println("\n✓ System Ready!");
  Serial.println("Waiting for car detections...\n");
}

void loop() {
  // Keep MQTT connection alive
  if (!mqttClient.connected()) {
    Serial.println("⚠️  MQTT disconnected. Reconnecting...");
    connectToMQTT();
  }
  mqttClient.poll();
  
  // Read data from ESP32-CAM
  if (Serial1.available() > 0) {
    String jsonData = Serial1.readStringUntil('\n');
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonData);
    
    if (!error) {
      bool car_detected = doc["sensors"]["car_detected"];
      float distance_cm = doc["sensors"]["distance_cm"];
      
      // Handle car detection
      if (car_detected && !gateIsOpen && carCounter < MAX_CAPACITY) {
        carCounter++;
        openGate();
        lastCarDetectionTime = millis();
        
        Serial.print("🚗 Car #");
        Serial.print(carCounter);
        Serial.println(" entered!");
        
        // Publish immediately when car detected
        publishParkingData(distance_cm, true);
      }
    }
  }
  
  // Auto-close gate
  if (gateIsOpen && (millis() - lastCarDetectionTime > gateOpenDuration)) {
    closeGate();
    Serial.println("🚪 Gate closed");
    publishParkingData(0, false);
  }
  
  // Publish periodic updates
  if (millis() - lastPublishTime > publishInterval) {
    publishParkingData(0, false);
    lastPublishTime = millis();
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  
  while (WiFi.begin(ssid, password) != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("\n✓ WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT broker");
  
  mqttClient.setUsernamePassword(mqtt_user, mqtt_password);
  
  while (!mqttClient.connect(mqtt_server, mqtt_port)) {
    Serial.print(".");
    delay(1000);
  }
  
  Serial.println("\n✓ Connected to MQTT Broker!");
}

void publishParkingData(float distance, bool carDetected) {
  StaticJsonDocument<256> doc;
  
  doc["car_count"] = carCounter;
  doc["capacity"] = MAX_CAPACITY;
  doc["gate_open"] = gateIsOpen;
  doc["available"] = MAX_CAPACITY - carCounter;
  doc["full"] = (carCounter >= MAX_CAPACITY);
  doc["distance_cm"] = distance;
  doc["car_detected"] = carDetected;
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Publish to MQTT
  mqttClient.beginMessage(topic_status);
  mqttClient.print(jsonString);
  mqttClient.endMessage();
  
  Serial.println("📤 Published: " + jsonString);
}

void openGate() {
  gateServo.write(GATE_OPEN_ANGLE);
  gateIsOpen = true;
}

void closeGate() {
  gateServo.write(GATE_CLOSED_ANGLE);
  gateIsOpen = false;
}