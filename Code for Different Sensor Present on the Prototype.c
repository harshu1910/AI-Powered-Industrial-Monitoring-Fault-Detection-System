
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <AccelStepper.h>

// --- Pin Definitions ---
#define DHT_PIN 4        // Temp and humidity sensor
#define MQ7_PIN 32       // CO sensor
#define MQ6_PIN 33       // LPG sensor
#define VOLTAGE_PIN 34   // Voltage sensor (Unused for motor display)
#define CURRENT_PIN 35   // Current sensor (Unused for motor display)
#define IR_PIN 25        // IR motion sensor
#define LDR_PIN 36       // Light sensor
#define LED1_PIN 26      // First LED
#define LED2_PIN 27      // Second LED
#define BUTTON_PIN 23    // Button to change screen
#define FLAME_PIN 13     // Flame sensor

// Stepper Motor Pin Definitions for ULN2003 Driver
#define MOTOR_IN1_PIN 14 // IN1 on ULN2003 board
#define MOTOR_IN2_PIN 12 // IN2 on ULN2003 board
#define MOTOR_IN3_PIN 15 // IN3 on ULN2003 board
#define MOTOR_IN4_PIN 2  // IN4 on ULN2003 board

#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
AccelStepper stepper(AccelStepper::FULL4WIRE, MOTOR_IN1_PIN, MOTOR_IN3_PIN, MOTOR_IN2_PIN, MOTOR_IN4_PIN);

// --- Global Variables ---
float temperature = 0.0;
float humidity = 0.0;
int mq7_co = 0;
int mq6_gas = 0;
float voltage = 0.0;
float current = 0.0;
bool ir_motion = false;
float light_level = 0.0;
bool flame_detected = false;

// State variables for controls
bool led1State = false;
bool led2State = false;
bool relay1State = false; // Controls the motor

unsigned long lastReadTime = 0;
const long readInterval = 500;
int oledScreenState = 0;
const int numScreens = 3;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastButtonState = HIGH;

// Draw everything on the OLED screen
void updateOledDisplay() {
  u8g2.clearBuffer();
  if (flame_detected) {
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(0, 20, "!! FLAME !!");
    u8g2.drawStr(0, 45, "!! DETECTED !!");
  } else {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (oledScreenState == 0) {
      u8g2.drawStr(2, 12, "Temp:");
      u8g2.setCursor(52, 12); u8g2.print(temperature, 1); u8g2.drawStr(92, 12, "C");
      u8g2.drawStr(2, 28, "Humi:");
      u8g2.setCursor(52, 28); u8g2.print(humidity, 1); u8g2.drawStr(92, 28, "%");
      u8g2.drawStr(2, 44, "CO:");
      u8g2.setCursor(52, 44); u8g2.print(mq7_co);
      u8g2.drawStr(2, 60, "LPG:");
      u8g2.setCursor(52, 60); u8g2.print(mq6_gas);
    } else if (oledScreenState == 1) {
      u8g2.drawStr(2, 12, "Volt:");
      u8g2.setCursor(52, 12); u8g2.print(voltage, 2); u8g2.drawStr(102, 12, "V");
      u8g2.drawStr(2, 28, "Curr:");
      u8g2.setCursor(52, 28); u8g2.print(current, 2); u8g2.drawStr(102, 28, "A");
      u8g2.drawStr(2, 44, "IR:");
      u8g2.drawStr(52, 44, ir_motion ? "DETECTED" : "CLEAR");
      u8g2.drawStr(2, 60, "Light:");
      u8g2.setCursor(52, 60); u8g2.print(light_level, 0); u8g2.drawStr(92, 60, "%");
    } else if (oledScreenState == 2) {
      u8g2.drawStr(2, 12, "Motor Status:");
      u8g2.drawStr(2, 28, relay1State ? "RUNNING" : "OFF");
      u8g2.drawStr(2, 44, "Motor Volt:");
      u8g2.setCursor(82, 44); u8g2.print(voltage, 2); u8g2.drawStr(112, 44, "V");
      u8g2.drawStr(2, 60, "Motor Curr:");
      u8g2.setCursor(82, 60); u8g2.print(current, 2); u8g2.drawStr(112, 60, "A");
    }
  }
  u8g2.sendBuffer();
}

// Read all sensors
void readAllSensors() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  mq7_co = analogRead(MQ7_PIN);
  mq6_gas = analogRead(MQ6_PIN);

  // Simulate motor voltage and current for display
  if (relay1State) {
    voltage = 5.0 + (random(0, 20) / 100.0); // 5.00V to 5.20V
    current = 0.4 + (random(0, 15) / 100.0); // 0.40A to 0.55A
  } else {
    voltage = 0.0;
    current = 0.0;
  }

  ir_motion = (digitalRead(IR_PIN) == LOW);
  int ldr_raw = analogRead(LDR_PIN);
  light_level = map(ldr_raw, 0, 4095, 0, 100);
  flame_detected = (digitalRead(FLAME_PIN) == LOW);
}

// CORS headers
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// Web Server: sensor data endpoint
void handleSensorData() {
  addCorsHeaders();
  StaticJsonDocument<512> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["mq7_co"] = mq7_co;
  doc["mq6_gas"] = mq6_gas;
  doc["voltage"] = voltage;
  doc["current"] = current;
  doc["ir_motion"] = ir_motion;
  doc["light_level"] = light_level;
  doc["flame_sensor"] = flame_detected;
  doc["led1"] = led1State;
  doc["led2"] = led2State;
  doc["relay1"] = relay1State;
  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

// Web Server: control endpoints
void handleControl(bool &stateVariable) {
  addCorsHeaders();
  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Body not received");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<64> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "text/plain", "JSON parsing failed");
    return;
  }
  stateVariable = doc["value"];
  server.send(200, "text/plain", "OK");
}

void handleLed1() { handleControl(led1State); }
void handleLed2() { handleControl(led2State); }
void handleRelay1() { handleControl(relay1State); }
void handleOptions() { addCorsHeaders(); server.send(204); }
void handleRoot() { addCorsHeaders(); server.send(200, "text/html", "<h1>ESP32 Sensor Hub</h1>"); }

// Setup
void setup() {
  Serial.begin(115200);
  u8g2.begin();
  dht.begin();
  pinMode(IR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FLAME_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT); digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT); digitalWrite(LED2_PIN, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  WiFiManager wm;
  wm.setConnectTimeout(30);
  bool res = wm.autoConnect("AutoConnectAP", "password");

  if (res) {
    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP Address: "); Serial.println(WiFi.localIP());
    u8g2.clearBuffer(); u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0, 15, "Connected!"); u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 40, "IP Address:");
    u8g2.drawStr(0, 55, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer(); delay(5000);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleSensorData);
    server.on("/led1", HTTP_POST, handleLed1);
    server.on("/led2", HTTP_POST, handleLed2);
    server.on("/relay1", HTTP_POST, handleRelay1);

    server.on("/data", HTTP_OPTIONS, handleOptions);
    server.on("/led1", HTTP_OPTIONS, handleOptions);
    server.on("/led2", HTTP_OPTIONS, handleOptions);
    server.on("/relay1", HTTP_OPTIONS, handleOptions);
    server.begin();
    Serial.println("HTTP server started");
  } else {
    Serial.println("Failed to connect to WiFi. Running in offline mode.");
    u8g2.clearBuffer(); u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 15, "WiFi Failed");
    u8g2.drawStr(0, 40, "Offline Mode");
    u8g2.sendBuffer();
    delay(3000);
  }
}

// Checks if the button is pressed to change OLED screens
void checkButton() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && lastButtonState == HIGH && millis() - lastDebounceTime > debounceDelay) {
    oledScreenState = (oledScreenState + 1) % numScreens;
    updateOledDisplay();
    lastDebounceTime = millis();
  }
  lastButtonState = reading;
}

// Main Loop
void loop() {
  checkButton();

  unsigned long currentMillis = millis();
  if (currentMillis - lastReadTime >= readInterval) {
    lastReadTime = currentMillis;
    readAllSensors();
    updateOledDisplay();
  }

  digitalWrite(LED1_PIN, led1State);
  digitalWrite(LED2_PIN, led2State);

  if (relay1State) {
    stepper.setSpeed(400);
  } else {
    stepper.setSpeed(0);
  }
  stepper.runSpeed();

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
}
