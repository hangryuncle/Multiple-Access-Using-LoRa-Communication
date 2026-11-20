
#include<Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <RTClib.h>   // only used for DateTime -> readable timestamp (no RTC hardware needed)
#include <limits.h>

#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

#define REPEATER_ID 1          // change for each repeater (2, 3, ...)

// Packet handling — accepts any packet length up to MAX_PACKET_SIZE.
#define MAX_PACKET_SIZE 255   // max LoRa payload size

/*
  Incoming packet (any length, little-endian fields when present):
    [0]     Node ID
    [1]     Packet ID
    [2-3]   Temperature x100 (int16,  INT16_MIN  = DHT fail)
    [4-5]   Humidity    x100 (uint16, UINT16_MAX = DHT fail)
    [6-9]   TX Unix timestamp (uint32)
    ...     any additional bytes are passed through unchanged

  Outgoing packet:
    [0 .. n-1]  Unchanged copy of received packet
    [n]         Repeater ID (appended)
*/

uint8_t rxData[MAX_PACKET_SIZE];
uint8_t txData[MAX_PACKET_SIZE + 1]; // +1 byte for appended repeater ID

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

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed! Check wiring.");
    while (1);
  }

  // Same settings as TX
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
  Serial.println("RX FORMAT = ANY LENGTH BINARY (up to 255 bytes)");
  Serial.println("TX FORMAT = RECEIVED BYTES + REPEATER ID");
  Serial.println("WAITING FOR PACKETS...");
  Serial.println("================================");
}

void loop() {

  int packetSize = LoRa.parsePacket();

  if (!packetSize) return;

  Serial.println("================================");
  Serial.println("PACKET RECEIVED");

  // Read packet
  int receivedLength = 0;

  while (LoRa.available()) {
    uint8_t b = LoRa.read();
    if (receivedLength < MAX_PACKET_SIZE) {
      rxData[receivedLength] = b;
    }
    receivedLength++;
  }

  // Cap to buffer size if the packet was larger than MAX_PACKET_SIZE
  if (receivedLength > MAX_PACKET_SIZE) {
    receivedLength = MAX_PACKET_SIZE;
  }

  // Accept packets of any length — no size filter.
  // NOTE: forwarding every packet can cause loops in a daisy-chained repeater
  // network. To prevent loops, ignore packets whose last byte is a known
  // repeater ID before retransmitting.

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // ---- Decode using the transmitter's format (only if long enough) ----
  if (receivedLength >= 10) {
    uint8_t nodeId   = rxData[0];
    uint8_t packetId = rxData[1];

    int16_t tempRaw = (int16_t)((uint16_t)rxData[2] | ((uint16_t)rxData[3] << 8));
    uint16_t humRaw = (uint16_t)rxData[4] | ((uint16_t)rxData[5] << 8);

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

  // ---- Build forwarded packet: received bytes + repeater ID ----
  memcpy(txData, rxData, receivedLength);
  txData[receivedLength] = REPEATER_ID;
  int txLength = receivedLength + 1;

  // Short forwarding delay
  delay(10);

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
