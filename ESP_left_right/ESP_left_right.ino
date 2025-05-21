// Code for LEFT-RIGHT

#include <WiFi.h>
#include <WiFiUdp.h>

// Network Configuration
const char* ssid = "ESP32_Hotspot";
const char* password = "12345678";

// Static IP Configuration for CLIENT ESP32s
IPAddress staticIP_left(192, 168, 4, 3);     // Left ESP32
IPAddress staticIP_right(192, 168, 4, 4);    // Right ESP32
IPAddress gateway(192, 168, 4, 1);           // AP's IP (beacon ESP32)
IPAddress subnet(255, 255, 255, 0);

// UDP Configuration
const char* masterIP = "192.168.4.2";        // Master ESP32's static IP
const int udpPort = 12345;

// Device Identification (Change this per device)
const char* deviceID = "ESP32-left";  // Change to "ESP32-right" for the right ESP

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  
  // Configure static IP based on device role
  if (strcmp(deviceID, "ESP32-left") == 0) {
    WiFi.config(staticIP_left, gateway, subnet);
  } else {
    WiFi.config(staticIP_right, gateway, subnet);
  }

  Serial.print("Connecting to WiFi as ");
  Serial.println(deviceID);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Master IP: ");
  Serial.println(masterIP);
  Serial.print("Gateway IP: ");
  Serial.println(gateway);
}

void loop() {
  // Read RSSI and send to master
  int rssi = WiFi.RSSI();
  String message = String(deviceID) + ":" + String(rssi);
  
  udp.beginPacket(masterIP, udpPort);
  udp.write((const uint8_t*)message.c_str(), message.length());
  udp.endPacket();

  Serial.print("Sent to master: ");
  Serial.println(message);
  
  delay(50);  // Adjusted delay for more stable communication
}