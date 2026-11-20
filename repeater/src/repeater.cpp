
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <RTClib.h>
#include <limits.h>
#include <string.h>

#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

#define REPEATER_ID 1

#define MAX_PACKET_SIZE 255

// Onboard status LED (ESP32 DevKit, active HIGH)
#define STATUS_LED       2
#define STATUS_LED_ON    HIGH
#define STATUS_LED_OFF   LOW
#define TX_LED_BLINK_MS  20

uint8_t rxData[MAX_PACKET_SIZE];
uint8_t txData[MAX_PACKET_SIZE + 1];

// Blink LED before every retransmission attempt
void blinkTxLed() {
  digitalWrite(STATUS_LED, STATUS_LED_ON);
  delay(TX_LED_BLINK_MS);
  digitalWrite(STATUS_LED, STATUS_LED_OFF);
}

void printHex(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize onboard LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, STATUS_LED_OFF);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed! Check wiring.");

    while (1) {
      digitalWrite(STATUS_LED, STATUS_LED_ON);
      delay(100);
      digitalWrite(STATUS_LED, STATUS_LED_OFF);
      delay(100);
    }
  }

  // Same settings as transmitter
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);
  LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setPreambleLength(16);
  LoRa.setSyncWord(0x12);

  Serial.println("================================");
  Serial.print("REPEATER ");
  Serial.print(REPEATER_ID);
  Serial.println(" READY");
  Serial.println("RX FORMAT = ANY LENGTH BINARY");
  Serial.println("TX FORMAT = RECEIVED BYTES + REPEATER ID");
  Serial.println("STATUS LED = BLINK ON TRANSMISSION");
  Serial.println("WAITING FOR PACKETS...");
  Serial.println("================================");
}

void loop() {

  int packetSize = LoRa.parsePacket();

  if (!packetSize) return;

  Serial.println("================================");
  Serial.println("PACKET RECEIVED");

  // Read received packet
  int receivedLength = 0;

  while (LoRa.available()) {
    uint8_t b = LoRa.read();

    if (receivedLength < MAX_PACKET_SIZE) {
      rxData[receivedLength] = b;
    }

    receivedLength++;
  }

  // Cap to buffer size
  if (receivedLength > MAX_PACKET_SIZE) {
    receivedLength = MAX_PACKET_SIZE;
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // Decode transmitter packet if at least 10 bytes
  if (receivedLength >= 10) {

    uint8_t nodeId = rxData[0];
    uint8_t packetId = rxData[1];

    int16_t tempRaw =
        (int16_t)((uint16_t)rxData[2] |
        ((uint16_t)rxData[3] << 8));

    uint16_t humRaw =
        (uint16_t)rxData[4] |
        ((uint16_t)rxData[5] << 8);

    uint32_t txTimestamp =
        (uint32_t)rxData[6] |
        ((uint32_t)rxData[7] << 8) |
        ((uint32_t)rxData[8] << 16) |
        ((uint32_t)rxData[9] << 24);

    Serial.print("Node ID: ");
    Serial.println(nodeId);

    Serial.print("Packet ID: ");
    Serial.println(packetId);

    if (tempRaw == INT16_MIN) {
      Serial.println("Temperature: DHT FAIL");
    } else {
      Serial.print("Temperature: ");
      Serial.println(tempRaw / 100.0f);
    }

    if (humRaw == UINT16_MAX) {
      Serial.println("Humidity: DHT FAIL");
    } else {
      Serial.print("Humidity: ");
      Serial.println(humRaw / 100.0f);
    }

    Serial.print("TX Timestamp: ");
    Serial.println(txTimestamp);

    Serial.print("TX Date and Time: ");
    Serial.println(DateTime(txTimestamp).timestamp());

  } else {
    Serial.println("Packet shorter than 10 bytes; skipping field decode.");
  }

  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("SNR: ");
  Serial.print(snr);
  Serial.println(" dB");

  Serial.print("Received Packet Size: ");
  Serial.print(receivedLength);
  Serial.println(" bytes");

  Serial.print("Raw HEX (RX): ");
  printHex(rxData, receivedLength);

  // Build forwarded packet
  memcpy(txData, rxData, receivedLength);

  // Append repeater ID
  txData[receivedLength] = REPEATER_ID;

  int txLength = receivedLength + 1;

  // Short forwarding delay
  delay(10);

  // Blink LED before retransmission
  blinkTxLed();

  // Retransmit packet
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write(txData, txLength);

  bool txOK = LoRa.endPacket();

  if (txOK) {

    Serial.println("PACKET RETRANSMITTED SUCCESSFULLY");

    Serial.print("Repeater ID: ");
    Serial.println(REPEATER_ID);

    Serial.print("Forwarded Packet Size: ");
    Serial.print(txLength);
    Serial.println(" bytes");

    Serial.print("Raw HEX (TX): ");
    printHex(txData, txLength);

  } else {
    Serial.println("RETRANSMISSION FAILED");
  }

  Serial.println("================================");
}
