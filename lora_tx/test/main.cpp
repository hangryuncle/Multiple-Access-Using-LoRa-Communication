
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <math.h>
#include <limits.h>
#include <WiFi.h>
#include "time.h"

#define DHTPIN   4
#define DHTTYPE  DHT22
DHT dht(DHTPIN, DHTTYPE);

RTC_DS3231 rtc;

#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

#define NODE_ID 1

#define PACKET_SIZE 14

uint8_t packet_id = 0;

const unsigned long SEND_INTERVAL = 20000;

// ---- WiFi / NTP settings (for syncing RTC at startup) ----
const char* WIFI_SSID     = "TP-Link_C9A8";
const char* WIFI_PASSWORD = "binay@5276";
const char* NTP_SERVER    = "pool.ntp.org";
const long  NEPAL_GMT_OFFSET_SEC = 20700;   // UTC+5:45
const int   NEPAL_DST_OFFSET_SEC = 0;       // Nepal has no DST
const unsigned long WIFI_TIMEOUT_MS = 15000UL;
const unsigned long NTP_TIMEOUT_MS  = 10000UL;

// Live time print (controlled by TIME / STOP commands)
bool livePrintEnabled = false;
int  lastPrintedSecond = -1;

// Converts RTC time to 12-hour format
void formatTime12(DateTime now, char *outBuf, size_t outBufSize) {

  int h12 = now.hour() % 12;

  if (h12 == 0)
    h12 = 12;

  const char *ampm = (now.hour() < 12) ? "AM" : "PM";

  snprintf(outBuf, outBufSize,
           "%02d:%02d:%02d %s",
           h12,
           now.minute(),
           now.second(),
           ampm);
}

// Send 14-byte binary packet
void sendSensorData() {

  packet_id++;

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  // Capture RTC timestamp
  DateTime now = rtc.now();

  // Add millisecond component
  uint64_t txTimestampMs =
      ((uint64_t)now.unixtime() * 1000ULL)
      + (millis() % 1000UL);

  uint8_t packet[PACKET_SIZE];

  // Byte 0: Node ID
  packet[0] = NODE_ID;

  // Byte 1: Packet ID
  packet[1] = packet_id;

  // Bytes 2-3: Temperature x100
  int16_t tempRaw;

  if (isnan(temp)) {
    tempRaw = INT16_MIN;
  } else {
    tempRaw = (int16_t)round(temp * 100.0f);
  }

  uint16_t tempData = (uint16_t)tempRaw;

  packet[2] = tempData & 0xFF;
  packet[3] = (tempData >> 8) & 0xFF;

  // Bytes 4-5: Humidity x100
  uint16_t humRaw;

  if (isnan(hum)) {
    humRaw = UINT16_MAX;
  } else {
    humRaw = (uint16_t)round(hum * 100.0f);
  }

  packet[4] = humRaw & 0xFF;
  packet[5] = (humRaw >> 8) & 0xFF;

  // Bytes 6-13: TX timestamp in milliseconds
  for (int i = 0; i < 8; i++) {
    packet[6 + i] =
        (txTimestampMs >> (8 * i)) & 0xFF;
  }

  // Transmit binary packet
  LoRa.beginPacket();
  LoRa.write(packet, sizeof(packet));

  bool txOk = LoRa.endPacket();

  Serial.println("================================");

  if (txOk) {

    Serial.println("BINARY PACKET SENT");

    Serial.print("Node ID: ");
    Serial.println(NODE_ID);

    Serial.print("Packet ID: ");
    Serial.println(packet_id);

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

    Serial.print("TX Date and Time: ");
    Serial.println(now.timestamp());

    Serial.print("Packet Size: ");
    Serial.print(sizeof(packet));
    Serial.println(" bytes");

    Serial.print("Raw HEX: ");

    for (int i = 0; i < sizeof(packet); i++) {
      if (packet[i] < 0x10)
        Serial.print("0");

      Serial.print(packet[i], HEX);
      Serial.print(" ");
    }

    Serial.println();

  } else {

    Serial.println("TX FAILED! Packet not sent.");
  }

  Serial.println("================================");
}

// Process RTC SET command
bool processTimeSetLine(const String &cmd) {

  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  if (!cmdUpper.startsWith("SET"))
    return false;

  int month, day, year;
  int hour12, minute, second;
  char ampm[3];

  int parsed = sscanf(
    cmd.c_str(),
    "%*s %d/%d/%d %d:%d:%d %2s",
    &month,
    &day,
    &year,
    &hour12,
    &minute,
    &second,
    ampm
  );

  if (parsed != 7 ||
      month < 1 || month > 12 ||
      day < 1 || day > 31 ||
      hour12 < 1 || hour12 > 12 ||
      minute < 0 || minute > 59 ||
      second < 0 || second > 59) {

    Serial.println(
      "Invalid SET format. Use: SET MM/DD/YYYY HH:MM:SS AM|PM"
    );

    return true;
  }

  String ampmStr = String(ampm);
  ampmStr.toUpperCase();

  if (ampmStr != "AM" && ampmStr != "PM") {

    Serial.println(
      "Invalid AM/PM value. Use AM or PM."
    );

    return true;
  }

  bool isPM = (ampmStr == "PM");

  int hour24 = hour12 % 12;

  if (isPM)
    hour24 += 12;

  rtc.adjust(
    DateTime(
      year,
      month,
      day,
      hour24,
      minute,
      second
    )
  );

  DateTime now = rtc.now();

  char timeStr[13];

  formatTime12(now, timeStr, sizeof(timeStr));

  Serial.print("RTC time set to: ");

  Serial.print(month);
  Serial.print("/");
  Serial.print(day);
  Serial.print("/");
  Serial.print(year);
  Serial.print(" ");

  Serial.println(timeStr);

  return true;
}

// Check Serial for RTC commands
void checkSerialForTimeSet() {

  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0)
    return;

  // TIME / STOP commands for live time printing
  String lineUpper = line;
  lineUpper.toUpperCase();

  if (lineUpper == "TIME") {

    livePrintEnabled = true;
    lastPrintedSecond = -1;

    Serial.println("Live time print STARTED. Send STOP to stop.");

    return;
  }

  if (lineUpper == "STOP") {

    livePrintEnabled = false;

    Serial.println("Live time print STOPPED.");

    return;
  }

  if (!processTimeSetLine(line)) {

    Serial.println(
      "Unrecognized command. Use: SET MM/DD/YYYY HH:MM:SS AM|PM"
    );
  }
}

// Set RTC from Serial at startup
void setRTCFromSerial() {

  const unsigned long WAIT_MS = 15000UL;

  unsigned long deadline = millis() + WAIT_MS;

  DateTime current = rtc.now();

  char curTimeStr[13];

  formatTime12(current, curTimeStr, sizeof(curTimeStr));

  Serial.println("================================");

  Serial.print("Current RTC time: ");

  Serial.print(current.month());
  Serial.print("/");
  Serial.print(current.day());
  Serial.print("/");
  Serial.print(current.year());
  Serial.print(" ");

  Serial.println(curTimeStr);

  Serial.println("================================");

  Serial.println("RTC TIME SET");

  Serial.println("Enter within 15 seconds:");

  Serial.println(
    "  SET MM/DD/YYYY HH:MM:SS AM|PM"
  );

  Serial.println(
    "  SKIP   (skip waiting, keep current RTC time)"
  );

  Serial.println(
    "No input -> keeps current RTC time after 15s."
  );

  Serial.println(
    "(You can also send SET anytime later.)"
  );

  Serial.println("================================");

  while (millis() < deadline) {

    if (Serial.available()) {

      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.length() == 0)
        continue;

      String cmdUpper = line;
      cmdUpper.toUpperCase();

      if (cmdUpper == "SKIP") {

        Serial.println(
          "SKIP received. Keeping current RTC time."
        );

        return;
      }

      if (processTimeSetLine(line))
        return;

      Serial.println(
        "Unrecognized command. Use SET ... or SKIP."
      );
    }

    delay(10);
  }

  Serial.println(
    "Timeout reached (15s). Keeping current RTC time."
  );
}

// Fetch Nepal time from the internet (NTP) and set the RTC
void setRTCFromNTP() {

  Serial.println("================================");
  Serial.println("Syncing RTC from internet (NTP)...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifiStart = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - wifiStart < WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("WiFi failed. Keeping current RTC time.");
    WiFi.mode(WIFI_OFF);
    return;
  }

  configTime(
    NEPAL_GMT_OFFSET_SEC,
    NEPAL_DST_OFFSET_SEC,
    NTP_SERVER
  );

  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, NTP_TIMEOUT_MS)) {

    Serial.println("NTP failed. Keeping current RTC time.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  rtc.adjust(
    DateTime(
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec
    )
  );

  DateTime synced = rtc.now();

  char syncStr[13];

  formatTime12(synced, syncStr, sizeof(syncStr));

  Serial.print("RTC synced to Nepal time: ");

  Serial.print(synced.month());
  Serial.print("/");
  Serial.print(synced.day());
  Serial.print("/");
  Serial.print(synced.year());
  Serial.print(" ");

  Serial.println(syncStr);

  // WiFi no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// Print current RTC time once per second while enabled
void printLiveTime() {

  if (!livePrintEnabled)
    return;

  DateTime t = rtc.now();

  if (t.second() == lastPrintedSecond)
    return;

  lastPrintedSecond = t.second();

  char liveStr[13];

  formatTime12(t, liveStr, sizeof(liveStr));

  Serial.print("[TIME] ");

  Serial.print(t.month());
  Serial.print("/");
  Serial.print(t.day());
  Serial.print("/");
  Serial.print(t.year());
  Serial.print(" ");

  Serial.println(liveStr);
}

void setup() {

  Serial.begin(115200);
  delay(100);

  dht.begin();

  Wire.begin();
  Wire.setClock(100000);

  if (!rtc.begin()) {

    Serial.println(
      "RTC not found! Check wiring."
    );

    while (1);
  }

  setRTCFromNTP();

  setRTCFromSerial();

  LoRa.setPins(
    LORA_SS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(LORA_FREQ)) {

    Serial.println(
      "LoRa init failed! Check wiring."
    );

    while (1);
  }

  LoRa.enableCrc();

  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);

  LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setPreambleLength(16);
  LoRa.setSyncWord(0x12);

  Serial.println("================================");
  Serial.println("TX1 READY");
  Serial.println("MODE = REPEATER TEST");
  Serial.println("PACKET FORMAT = 14 BYTE BINARY");
  Serial.println("TIMESTAMP = MILLISECONDS");
  Serial.println("PACKET INTERVAL = 20 sec");
  Serial.println("PACKETS PER MINUTE = 3");
  Serial.println("================================");
}

void loop() {

  checkSerialForTimeSet();

  printLiveTime();

  static unsigned long lastSend = 0;

  unsigned long currentMillis = millis();

  if (lastSend == 0 ||
      currentMillis - lastSend >= SEND_INTERVAL) {

    lastSend = currentMillis;

    sendSensorData();
  }

  delay(100);
}
