#include <Arduino.h>

// Right motors :-
#define L_LPWM 18
#define L_RPWM 5

// Left motors :-
#define R_LPWM  21
#define R_RPWM  19

// #define EN 2

#define VX 35  // X-axis pin
#define VY 34  // Y-axis pin

float mapFloat(float t, float in_min, float in_max, float out_min, float out_max) {
  return (t - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Anticlock condition --> RPWM = 0 ; LPWM = value
// Clock condition --> RPWM = value ; LPWM = 0

void forward(int value1, int value2) {
  // Left motors -- Anticlock
  analogWrite(L_LPWM, value1);
  analogWrite(L_RPWM, 0);
  // right motors -- Clock
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, value2);
}

void backward(int value1, int value2) {
  // Left motors -- Clock
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, value1);
  // right motors -- Anticlock
  analogWrite(R_LPWM, value2);
  analogWrite(R_RPWM, 0);
}

void setup() {

  pinMode(L_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  // pinMode(EN, OUTPUT);

  pinMode(VX, INPUT);
  pinMode(VY, INPUT);

  Serial.begin(115200);
  Serial.println("...ESP32 has been started...");
}

void loop() {
  int Vx_value = analogRead(VX);
  int Vy_value = analogRead(VY);
  int VIN = analogRead(2);
  float Vin_mod = mapFloat(VIN, 0, 4095, 0, 255);
  Serial.print("Vin value:- ");
  Serial.println(Vin_mod);

  float Vx_mod = mapFloat(Vx_value, 0, 4095, -200, 200);
  float Vy_mod = mapFloat(Vy_value, 0, 4095, 50, -50);

  // Serial.print("Vx :- ");
  // Serial.print(Vx_mod);
  // Serial.print("  |  ");
  // Serial.print("Vy :- ");
  // Serial.println(Vy_mod);
  // if (Vx_mod>20){
  //   // forward(Vx_mod-Vy_mod, Vx_mod+Vy_mod);
  //   forward(Vin_mod, Vin_mod);
  //   Serial.println("Forward");
  // }
  // else if (Vx_mod<-20){
  //   Vx_mod = abs(Vx_mod);
  //   // backward(Vx_mod-Vy_mod, Vx_mod+Vy_mod);
  //   backward(Vin_mod, Vin_mod);
  //   Serial.println("Backward");
  // }
  forward(Vin_mod, Vin_mod);
  delay(50);
}
