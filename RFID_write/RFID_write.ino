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
  Serial.println("Place card to write data...");
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
  byte dataToWrite[16] = "6969"; // Data to write

  if (rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &rfid.uid) != MFRC522::STATUS_OK) {
    Serial.println("Authentication failed");
    return;
  }

  if (rfid.MIFARE_Write(block, dataToWrite, 16) == MFRC522::STATUS_OK) {
    Serial.println("Data written to tag successfully");
  } else {
    Serial.println("Write failed");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(1000);
}
