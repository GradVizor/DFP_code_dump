// Code for BEACON

#include <WiFi.h>

const char* ssid = "ESP32_Hotspot";
const char* password = "12345678";

// Define static IP for the AP
IPAddress local_IP(192, 168, 4, 1);    // Default ESP32 AP IP (can be changed)
IPAddress gateway(192, 168, 4, 1);     // Same as local_IP for AP mode
IPAddress subnet(255, 255, 255, 0);    // Standard subnet mask

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure static IP for SoftAP
  WiFi.softAPConfig(local_IP, gateway, subnet);

  // Start AP with SSID and password
  WiFi.softAP(ssid, password);

  Serial.println("ESP32 Hotspot created");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.softAPIP());  // Should print 192.168.4.1 (or your chosen IP)
}

void loop() {
  // Keep the AP running
}