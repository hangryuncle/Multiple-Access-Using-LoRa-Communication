
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

// Onboard status LED (ESP32 DevKit: GPIO 2, active HIGH).
// A transmission attempt is marked by a 10 ms blink.
#define STATUS_LED       2
#define STATUS_LED_ON    HIGH
#define STATUS_LED_OFF   LOW
#define TX_LED_BLINK_MS  20

uint8_t packet_id = 0;

// ---- WiFi / NTP settings (for syncing RTC at startup) ----
// Credentials live in include/secrets.h so they stay out of main source.
#if __has_include("secrets.h")
  #include "secrets.h"
#endif

// Fallbacks keep the build valid if secrets.h is missing.
// With empty credentials the NTP sync simply times out gracefully.
#ifndef WIFI_SSID
  #define WIFI_SSID     ""
#endif

#ifndef WIFI_PASSWORD
  #define WIFI_PASSWORD ""
#endif

const char* NTP_SERVER    = "pool.ntp.org";
const long  NEPAL_GMT_OFFSET_SEC = 20700;
const int   NEPAL_DST_OFFSET_SEC = 0;
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

// Briefly blink the onboard LED to indicate a transmission attempt.
// Blocks for TX_LED_BLINK_MS only (10 ms), which is negligible next to the
// ~2-3 s LoRa airtime at SF12 / BW 62.5 kHz.
void blinkTxLed() {

  digitalWrite(STATUS_LED, STATUS_LED_ON);

  delay(TX_LED_BLINK_MS);

  digitalWrite(STATUS_LED, STATUS_LED_OFF);
}

// Halt with a visible indication instead of hanging silently.
// Blinks the status LED quickly for 5 s, then reboots the ESP32.
void fatalError(const char *message) {

  Serial.println(message);
  Serial.println("HALTED - restarting in 5 seconds...");

  unsigned long start = millis();

  while (millis() - start < 5000UL) {

    digitalWrite(STATUS_LED, STATUS_LED_ON);
    delay(100);

    digitalWrite(STATUS_LED, STATUS_LED_OFF);
    delay(100);
  }

  ESP.restart();
}

// ---- Sub-second RTC alignment ----
// millis() runs independently of the RTC, so (millis() % 1000) has no
// relation to the RTC's second boundary. Instead we anchor millis() to each
// observed RTC second edge and derive the millisecond fraction from it.
uint32_t subSecondMs    = 0;
uint32_t lastRtcSecond   = 0;
uint32_t millisAtRtcEdge = 0;

// Wait (bounded) for the next RTC second edge so the fraction starts out
// phase-locked instead of at an arbitrary point in the second.
void syncSubSecondClock() {

  uint32_t second = rtc.now().unixtime();
  unsigned long start = millis();

  while (rtc.now().unixtime() == second &&
         millis() - start < 1500UL) {
    // waiting for the RTC tick
  }

  millisAtRtcEdge = millis();
  lastRtcSecond = rtc.now().unixtime();
  subSecondMs = 0;
}

// Serviced every loop, and again right before a packet is built.
void updateSubSecondClock(const DateTime &now) {

  uint32_t nowMs = millis();

  if (now.unixtime() != lastRtcSecond) {
    lastRtcSecond = now.unixtime();
    millisAtRtcEdge = nowMs;
  }

  uint32_t fraction = nowMs - millisAtRtcEdge;

  // A long blocking section can delay the next edge observation; clamp
  // rather than emit a nonsense value.
  subSecondMs = (fraction > 999UL) ? 999UL : fraction;
}

// Transmit a packet, retrying if an attempt fails.
// Each attempt blinks the status LED. Returns true on success.
bool sendPacketWithRetry(const uint8_t *data, size_t len, uint8_t maxAttempts) {

  for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {

    blinkTxLed();

    LoRa.idle();  // SX127x must be in standby before starting a TX
    LoRa.beginPacket();
    LoRa.write(data, len);

    if (LoRa.endPacket()) {
      return true;
    }

    Serial.print("TX attempt ");
    Serial.print(attempt);
    Serial.println(" failed.");

    if (attempt < maxAttempts) {
      delay(200);
    }
  }

  return false;
}

// Send 14-byte binary packet
void sendSensorData() {

  packet_id++;

  // Capture the timestamp before the (~250 ms) DHT read so the millisecond
  // field matches packet build time, not sensor completion time.
  DateTime now = rtc.now();
  updateSubSecondClock(now);

  uint64_t txTimestampMs =
      ((uint64_t)now.unixtime() * 1000ULL)
      + subSecondMs;

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

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

  // Transmit binary packet, with one retry if the first attempt fails.
  bool txOk = sendPacketWithRetry(packet, sizeof(packet), 2);

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
  char ampm[3] = {0};

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

// Non-blocking serial line reader.
// Returns true only when a complete, non-empty line has arrived.
// It never waits for bytes, so the TDMA timing is not disturbed by Serial.
const size_t SERIAL_LINE_MAX = 63;
char   serialLineBuf[SERIAL_LINE_MAX + 1];
size_t serialLineLen = 0;

bool readSerialLine(String &line) {

  while (Serial.available() > 0) {

    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c != '\n') {

      if (serialLineLen < SERIAL_LINE_MAX) {
        serialLineBuf[serialLineLen++] = c;
      }

      continue;
    }

    serialLineBuf[serialLineLen] = '\0';
    serialLineLen = 0;

    line = serialLineBuf;
    line.trim();

    if (line.length() > 0) {
      return true;
    }
  }

  return false;
}

// Check Serial for RTC commands
void checkSerialForTimeSet() {

  String line;

  if (!readSerialLine(line)) {
    return;
  }

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

  unsigned long waitStart = millis();

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

  while (millis() - waitStart < WAIT_MS) {

    String line;

    if (readSerialLine(line)) {

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

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, STATUS_LED_OFF);

  dht.begin();

  Wire.begin();
  Wire.setClock(100000);

  if (!rtc.begin()) {
    fatalError("RTC not found! Check wiring.");
  }

  setRTCFromNTP();

  setRTCFromSerial();

  syncSubSecondClock();

  LoRa.setPins(
    LORA_SS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(LORA_FREQ)) {
    fatalError("LoRa init failed! Check wiring.");
  }

  LoRa.enableCrc();

  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);

  LoRa.setTxPower(18, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setPreambleLength(16);
  LoRa.setSyncWord(0x12);

  Serial.println("================================");
  Serial.println("TX1 READY");
  Serial.println("MODE = TDMA");
  Serial.println("PACKET FORMAT = 14 BYTE BINARY");
  Serial.println("TIMESTAMP = MILLISECONDS");
  Serial.println("TX1 SLOT = 0-18 sec");
  Serial.println("TRANSMISSION = BEGINNING OF SLOT");
  Serial.println("PACKETS PER MINUTE = 1");
  Serial.println("================================");
}

void loop() {

  checkSerialForTimeSet();

  printLiveTime();

  static uint32_t lastSentMinute = UINT32_MAX;

  DateTime now = rtc.now();

  updateSubSecondClock(now);

  uint32_t currentMinute = now.unixtime() / 60;
   

  // TX1 transmission window: 0-5 seconds
  // Transmit only once per minute
  if (now.second() < 5 &&
      lastSentMinute != currentMinute) {

    // Lock this minute before transmitting
    lastSentMinute = currentMinute;


    sendSensorData();
  }

  delay(10);
}

