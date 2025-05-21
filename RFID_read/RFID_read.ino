#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 15
#define RST_PIN 5

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, 15); // SCK, MISO, MOSI, SS
  rfid.PCD_Init();

  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  Serial.println("Place card to read data...");
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  Serial.print("Card UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  byte block = 4;
  byte buffer[18];
  byte size = sizeof(buffer);

  if (rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &rfid.uid) != MFRC522::STATUS_OK) {
    Serial.println("Authentication failed");
    return;
  }

  if (rfid.MIFARE_Read(block, buffer, &size) == MFRC522::STATUS_OK) {
    Serial.print("Read Data: ");
    for (byte i = 0; i < 16; i++) {
      Serial.write(buffer[i]);
    }
    Serial.println();
  } else {
    Serial.println("Read failed");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(1000);
}
