// ESP32 Master Controller - Complete with Corrected Motor Directions
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

// Motor Control Pins (Corrected Directions)
#define R_LPWM 21         // Right motor forward (pin 18)
#define R_RPWM 19          // Right motor reverse (pin 5)
#define L_LPWM 18         // Left motor forward (pin 21)
#define L_RPWM 5         // Left motor reverse (pin 19)
#define EN 2              // Motor enable pin

// Joystick Pins
#define VX 35             // X-axis (left/right)
#define VY 34             // Y-axis (forward/backward)

// Configuration Parameters
#define MARGIN 5          // RSSI difference threshold for turning
#define THRESHOLD 55      // Minimum RSSI for movement
#define MOTOR_THRESHOLD 190 // Minimum motor speed
#define MOTOR_THRESHOLD_2 210 // Minimum motor speed

#define MODE_TIMEOUT 1000 // 1s timeout for auto->manual transition
#define RSSI_FILTER_SIZE 5 // Number of samples for RSSI filtering
#define MAX_MOTOR_SPEED 250 // Maximum PWM value
#define JOYSTICK_DEADZONE 30 // Deadzone to prevent drift

// Network Configuration
const char* ssid = "ESP32_Hotspot";
const char* password = "12345678";

// Static IP Configuration
IPAddress staticIP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

WiFiUDP udp;
const int udpPort = 12345;
char incomingPacket[255];

// System Variables
int rssi_left = 0;
int rssi_right = 0;
int rssi_self = 0;
unsigned long lastPacketTime = 0;
int rssi_left_history[RSSI_FILTER_SIZE] = {0};
int rssi_right_history[RSSI_FILTER_SIZE] = {0};
int filter_index = 0;
bool autoMode = true;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void applyMotorSpeed(int left, int right) {
  // Constrain speeds to safe limits
  left = constrain(left, 0, MAX_MOTOR_SPEED);
  right = constrain(right, 0, MAX_MOTOR_SPEED);

  // Left motor control (corrected)
  if (left > 0) {
    analogWrite(L_LPWM, left);
    analogWrite(L_RPWM, 0);
  } else {
    analogWrite(L_LPWM, 0);
    analogWrite(L_RPWM, left);
  }

  // Right motor control (corrected)
  if (right > 0) {
    analogWrite(R_LPWM, 0);
    analogWrite(R_RPWM, right);
  } else {
    analogWrite(R_LPWM, right);
    analogWrite(R_RPWM, 0);
  }
}

void forward(){
  // Both motors forward
  analogWrite(L_LPWM, MOTOR_THRESHOLD_2);
  analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, MOTOR_THRESHOLD_2);
}

void rotate(){
  // Both motors forward
  analogWrite(L_LPWM, MOTOR_THRESHOLD);
  analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, MOTOR_THRESHOLD);
  analogWrite(R_RPWM, 0);
}

void left(){
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, 0);
  // right motors -- Clock
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, MOTOR_THRESHOLD_2);
}

void right() {
  // Left motors -- Anticlock
  analogWrite(L_LPWM, MOTOR_THRESHOLD_2);
  analogWrite(L_RPWM, 0);
  // right motors -- Clock
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, 0);
}

void stop() {
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, 0);
}

void handleJoystick() {
  int Vx_value = analogRead(VX);
  int Vy_value = analogRead(VY);

  // Map joystick values to motor speed range with deadzone
  float Vx_mod = mapFloat(Vx_value, 0, 4095, -125, 125);
  float Vy_mod = mapFloat(Vy_value, 0, 4095, 125, -125);

  // Apply deadzone
  if (abs(Vx_mod) < JOYSTICK_DEADZONE && abs(Vy_mod) < JOYSTICK_DEADZONE) {
    stop();
    return;
  }

  // Calculate motor speeds (corrected directions)
  int left_speed = MOTOR_THRESHOLD + Vx_mod - Vy_mod;  // Note sign change
  int right_speed = MOTOR_THRESHOLD + Vx_mod + Vy_mod; // Note sign change

  // Constrain speeds and apply
  left_speed = constrain(left_speed, 0, MAX_MOTOR_SPEED);
  right_speed = constrain(right_speed, 0, MAX_MOTOR_SPEED);
  applyMotorSpeed(left_speed, right_speed);

  Serial.print("Manual Control - L:");
  Serial.print(left_speed);
  Serial.print(" R:");
  Serial.println(right_speed);
}

void handleAutoMode() {
  // Calculate filtered RSSI values
  int filtered_left = 0, filtered_right = 0;
  for (int i = 0; i < RSSI_FILTER_SIZE; i++) {
    filtered_left += abs(rssi_left_history[i]);
    filtered_right += abs(rssi_right_history[i]);
  }
  filtered_left /= RSSI_FILTER_SIZE;
  filtered_right /= RSSI_FILTER_SIZE;

  Serial.print("Filtered left :-");
  Serial.println(filtered_left);

  int error = filtered_left - filtered_right;
  int rssi_RL = (filtered_left + filtered_right) / 2;

  // Movement logic with corrected directions
  if (error > MARGIN) {
    right(); // Stronger signal on left -> turn left (toward left beacon)
    Serial.print("Right turn - Error:");
    Serial.println(error);
  } 
  else if (error < -MARGIN) {
    left(); // Stronger signal on right -> turn right (toward right beacon)
    Serial.print("Left turn - Error:");
    Serial.println(error);
  }
  else {
    if (filtered_left > THRESHOLD && filtered_right > THRESHOLD) { //   && !digitalRead(SENSOR_FRONT)
      if (rssi_self < rssi_RL){
        Serial.println("............FORWARD.............");
        forward();
      }
      else if (rssi_self > rssi_RL) {
        Serial.println("............BACKWARD_STOP.............");
        stop();
      }
    } 
    else {
      Serial.println("............STOP.............");
      stop();
    }
  }
}

void setup() {
  // Initialize hardware with corrected pin assignments
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  digitalWrite(L_LPWM, LOW);
  digitalWrite(L_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);
  digitalWrite(R_RPWM, LOW);

  pinMode(VX, INPUT);
  pinMode(VY, INPUT);

  Serial.begin(115200);
  Serial.println("Initializing with corrected motor directions...");

  // Configure WiFi
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure static IP");
  }

  // Connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    // Serial.print(".");
    handleJoystick();
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  udp.begin(udpPort);
  Serial.print("UDP server started on port ");
  Serial.println(udpPort);
}

void loop() {
  // Update current RSSI
  rssi_self = abs(WiFi.RSSI());

  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    lastPacketTime = millis();
    autoMode = true;
    
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
      String data = String(incomingPacket);
      
      int separatorIndex = data.indexOf(':');
      if (separatorIndex != -1) {
        String deviceID = data.substring(0, separatorIndex);
        int rssi = data.substring(separatorIndex + 1).toInt();

        if (deviceID == "ESP32-left") {
          rssi_left_history[filter_index] = abs(rssi);
          rssi_left = rssi_left_history[filter_index];
        } 
        else if (deviceID == "ESP32-right") {
          rssi_right_history[filter_index] = abs(rssi);
          rssi_right = rssi_right_history[filter_index];
        }
        
        filter_index = (filter_index + 1) % RSSI_FILTER_SIZE;
      }
    }
  }

  // Mode selection logic
  if (autoMode) {
    if (millis() - lastPacketTime > MODE_TIMEOUT) {
      autoMode = false;
      Serial.println("Switching to MANUAL mode");
      stop();
    } else {
      handleAutoMode();
    }
  } else {
    handleJoystick();
  }

  delay(10);
}