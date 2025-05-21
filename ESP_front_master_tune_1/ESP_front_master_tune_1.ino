// Code for the MASTER

#include <WiFi.h>
#include <WiFiUdp.h>
#include <cmath>
#include <Arduino.h>

// Sensors :-
#define SENSOR_LEFT 32
#define SENSOR_RIGHT 33
#define SENSOR_FRONT 23

// Right motors :-
#define L_LPWM 18
#define L_RPWM 5

// Left motors :-
#define R_LPWM  21
#define R_RPWM  19

#define EN 2

#define VX 35  // X-axis pin
#define VY 34  // Y-axis pin

// Values used further
#define MARGIN 5
#define THRESHOLD 55
#define MOTOR_THRESHOLD 90

// Network configuration
const char* ssid = "ESP32_Hotspot";
const char* password = "12345678";

// Static IP configuration for the MASTER ESP32
IPAddress staticIP(192, 168, 4, 2);      // Master's static IP
IPAddress gateway(192, 168, 4, 1);       // AP's IP (beacon ESP32)
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);        // Optional
IPAddress secondaryDNS(8, 8, 4, 4);      // Optional

WiFiUDP udp;
const int udpPort = 12345;
char incomingPacket[255];

// Variables to store RSSI values
int rssi_left = 0;
int rssi_right = 0;
int rssi_self = 0;

float mapFloat(float t, float in_min, float in_max, float out_min, float out_max) {
  return (t - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void forward(int value1, int value2) {
  // Left motors -- Anticlock
  analogWrite(L_LPWM, MOTOR_THRESHOLD + value1);
  analogWrite(L_RPWM, 0);
  // right motors -- Clock
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, MOTOR_THRESHOLD + value2);
}

void backward(int value1, int value2) {
  // Left motors -- Clock
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, value1);
  // right motors -- Anticlock
  analogWrite(R_LPWM, value2);
  analogWrite(R_RPWM, 0);
}

void rotate(int value) {
  // Left motors -- Anticlock
  analogWrite(L_LPWM, MOTOR_THRESHOLD + value);
  analogWrite(L_RPWM, 0);
  // right motors -- Anticlock
  analogWrite(R_LPWM, MOTOR_THRESHOLD + value);
  analogWrite(R_RPWM, 0);
}

void stop() {
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, 0);
}

void joystick() {
  int Vx_value = analogRead(VX);
  int Vy_value = analogRead(VY);

  float Vx_mod = mapFloat(Vx_value, 0, 4095, -125, 125);
  float Vy_mod = mapFloat(Vy_value, 0, 4095, 125, -125);

  Serial.print("Vx :- ");
  Serial.print(Vx_mod);
  Serial.print("  |  ");
  Serial.print("Vy :- ");
  Serial.println(Vy_mod);

  if (Vx_mod>0){
    forward(Vx_mod+Vy_mod, Vx_mod-Vy_mod);
    // forward(255,255);
    Serial.println("Forward");
  }
  else if (Vx_mod<0){
    Vx_mod = abs(Vx_mod);
    backward(Vx_mod-Vy_mod, Vx_mod+Vy_mod);
    // backward(255,255);
    Serial.println("Backward");
  }
  delay(50);
}

void setup() {

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);

  // Initialize motor control pins
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(EN, OUTPUT);

  pinMode(VX, INPUT);
  pinMode(VY, INPUT);

  Serial.begin(115200);
  
  // Configure static IP
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    // delay(1000);
    // Serial.print(".");
    joystick();
    // delay(50);
  }
  
  Serial.println("\nConnected to WiFi.");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Verify static IP assignment
  if (WiFi.localIP() != staticIP) {
    Serial.println("Warning: Failed to set static IP! Using DHCP-assigned IP.");
  }

  // Start UDP
  udp.begin(udpPort);
  Serial.print("Listening on UDP port ");
  Serial.println(udpPort);
  
}

void loop() {
  rssi_self = abs(WiFi.RSSI());
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    // Read the packet
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
  
    // Extract RSSI value and device ID
    String data = String(incomingPacket);
    int separatorIndex = data.indexOf(':');
    if (separatorIndex != -1) {
      String deviceID = data.substring(0, separatorIndex);
      int rssi = data.substring(separatorIndex + 1).toInt();

      // Store RSSI value based on device ID
      if (deviceID == "ESP32-left") {
        rssi_left = abs(rssi);
      } else if (deviceID == "ESP32-right") {
        rssi_right = abs(rssi);
      }

      int error = rssi_left - rssi_right; // +ve--> right & -ve--> left
      int rssi_RL = (rssi_left + rssi_right)/2;
      int rssi_threshold = rssi_RL - rssi_self; // +ve--> forward & -ve--> backward

      if (rssi_left != 0 && rssi_right != 0) {
        Serial.print("Left RSSI: ");
        Serial.print(rssi_left);
        Serial.print(" | Right RSSI: ");
        Serial.print(rssi_right);
        Serial.print(" | Self RSSI: ");
        Serial.println(rssi_self);

        if (error > MARGIN) { //  && !digitalRead(SENSOR_RIGHT)
            Serial.println("............RIGHT.............");
            forward(abs(error)*3,abs(error)*1);
        } 
        else if (error < -MARGIN) { //   && !digitalRead(SENSOR_LEFT)
            Serial.println("............LEFT.............");
            forward(abs(error)*1,abs(error)*3);
        } 
        else {
          if (rssi_left > THRESHOLD && rssi_right > THRESHOLD) { //   && !digitalRead(SENSOR_FRONT)
            if (rssi_self < rssi_RL){
              Serial.println("............FORWARD.............");
              forward(abs(error)*3,abs(error)*3);
            }
            else if (rssi_self > rssi_RL) {
              Serial.println("............BACKWARD.............");
              rotate(abs(error)*3);
            }
          } 
          else {
            Serial.println("............STOP.............");
            stop();
          }
        }

      }
    }
  }
  else{
    joystick();
  }
  delay(10);
}