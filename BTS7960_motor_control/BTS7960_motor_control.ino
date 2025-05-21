#include <Arduino.h>

// Left motors :-
#define L_LPWM 18
#define L_RPWM 5
#define R_LPWM 21
#define R_RPWM 19

#define EN 2


void backward(int value) {
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, value);
  analogWrite(R_LPWM, value);
  analogWrite(R_RPWM, 0);
}

void forward(int value) {
  analogWrite(L_LPWM, value);
  analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, value);
}
void stop() {
  analogWrite(L_LPWM, 0);
  analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0);
  analogWrite(R_RPWM, 0);
}

float mapFloat(float t, float in_min, float in_max, float out_min, float out_max) {
  return (t - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {

  pinMode(EN, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  // pinMode(35, OUTPUT);

  digitalWrite(EN, HIGH);

  Serial.begin(115200);
  Serial.println("...ESP32 has been started...");
}

void loop() {
  // int VIN = analogRead(35);
  // float input = mapFloat(VIN, 0, 4095, 0, 255);
  // Serial.print("Vin value:- ");
  // Serial.println(input);
  int input = 250; 

  forward(input);
  Serial.println("__________ANTI-CLOCKWISE__________");
  delay(3000);

  stop();
  Serial.println("______________STOP________________");
  delay(3000);

  backward(input);
  Serial.println("__________CLOCKWISE__________");
  delay(3000);

  stop();
  Serial.println("______________STOP________________");
  delay(2000);
}
