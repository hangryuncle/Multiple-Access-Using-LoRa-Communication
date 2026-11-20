
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <Wire.h>
#include <RTClib.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include "time.h"

#include "secrets.h"

// ============================================================
// LoRa configuration
// ============================================================

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23

#define LORA_FREQ 433E6

// 14-byte packet format
//
// Byte 0       : Node ID
// Byte 1       : Packet ID
// Bytes 2-3    : Temperature x100 (int16)
// Bytes 4-5    : Humidity x100 (uint16)
// Bytes 6-13   : TX Unix timestamp in milliseconds (uint64)

#define PACKET_SIZE 14

RTC_DS3231 rtc;

uint8_t receivedData[PACKET_SIZE];

// ============================================================
// Wi-Fi / API configuration
// ============================================================

//API_URL is defined in secrets.h

// ============================================================
// LittleFS
// ============================================================

const char *QUEUE_FILE = "/queue.txt";

// ============================================================
// Preferences
// ============================================================

Preferences preferences;

// ============================================================
// Wi-Fi state
// ============================================================

String savedSSID;
String savedPassword;

// ============================================================
// Serial timeout
// ============================================================

const unsigned long WIFI_INPUT_TIMEOUT = 60000;

// ============================================================
// NTP settings
// ============================================================

const char *NTP_SERVER = "pool.ntp.org";
const long NEPAL_GMT_OFFSET_SEC = 20700;
const int NEPAL_DST_OFFSET_SEC = 0;
const unsigned long NTP_TIMEOUT_MS = 10000UL;

// ============================================================
// Live time print
// ============================================================

bool livePrintEnabled = false;
int lastPrintedSecond = -1;

// ============================================================
// Function declarations
// ============================================================

bool connectWiFi();

bool configureWiFi();

void startupWiFiSetup();

void processSerialCommands();

void receiveLoRaPacket();

bool sendToAPI(const String &packet, int rssi, float snr);

bool sendQueuedPackets();

bool saveToQueue(const String &packet, int rssi, float snr);

bool convertLoRaToAPI(
    const String &input,
    String &output,
    int rssi,
    float snr);

bool convertQueuedPacket(
    const String &input,
    String &output);

bool isAPIFormat(const String &packet);

bool queueFileExists();

void printWiFiStatus();

void printLiveTime();

void setRTCFromNTP();

// RTC functions

void formatTime12(DateTime now, char *outBuf, size_t outBufSize);

void formatTimestampMs(
    uint64_t timestampMs,
    char *outBuf,
    size_t outBufSize);

bool processTimeSetLine(const String &cmd);

void checkSerialForTimeSet();

void setRTCFromSerial();

// ============================================================
// Format timestamp with milliseconds
// ============================================================

void formatTimestampMs(
    uint64_t timestampMs,
    char *outBuf,
    size_t outBufSize)
{
    uint32_t timestampSeconds =
        (uint32_t)(timestampMs / 1000ULL);

    uint16_t milliseconds =
        (uint16_t)(timestampMs % 1000ULL);

    DateTime time(timestampSeconds);

    snprintf(
        outBuf,
        outBufSize,
        "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
        time.year(),
        time.month(),
        time.day(),
        time.hour(),
        time.minute(),
        time.second(),
        milliseconds
    );
}

// ============================================================
// RTC 12-hour formatting
// ============================================================

void formatTime12(
    DateTime now,
    char *outBuf,
    size_t outBufSize)
{
    int h12 = now.hour() % 12;

    if (h12 == 0)
        h12 = 12;

    const char *ampm =
        (now.hour() < 12) ? "AM" : "PM";

    snprintf(
        outBuf,
        outBufSize,
        "%02d:%02d:%02d %s",
        h12,
        now.minute(),
        now.second(),
        ampm
    );
}

// ============================================================
// Process RTC SET command
//
// Format:
// SET MM/DD/YYYY HH:MM:SS AM|PM
// ============================================================

bool processTimeSetLine(const String &cmd)
{
    String cmdUpper = cmd;
    cmdUpper.trim();
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

    if (
        parsed != 7 ||
        month < 1 || month > 12 ||
        day < 1 || day > 31 ||
        year < 2000 || year > 2099 ||
        hour12 < 1 || hour12 > 12 ||
        minute < 0 || minute > 59 ||
        second < 0 || second > 59
    )
    {
        Serial.println(
            "Invalid SET format. Use: SET MM/DD/YYYY HH:MM:SS AM|PM"
        );
        return true;
    }

    String ampmStr = String(ampm);
    ampmStr.toUpperCase();

    if (ampmStr != "AM" && ampmStr != "PM")
    {
        Serial.println("Invalid AM/PM value.");
        return true;
    }

    int hour24 = hour12 % 12;

    if (ampmStr == "PM")
        hour24 += 12;

    DateTime newTime(
        year,
        month,
        day,
        hour24,
        minute,
        second
    );

    // Validate the calendar date

    if (
        newTime.year() != year ||
        newTime.month() != month ||
        newTime.day() != day
    )
    {
        Serial.println("Invalid calendar date.");
        return true;
    }

    rtc.adjust(newTime);

    DateTime now = rtc.now();

    char timeStr[13];

    formatTime12(now, timeStr, sizeof(timeStr));

    Serial.println("========================================");
    Serial.println("RTC TIME UPDATED");

    Serial.print("Date: ");
    Serial.print(now.month());
    Serial.print("/");
    Serial.print(now.day());
    Serial.print("/");
    Serial.println(now.year());

    Serial.print("Time: ");
    Serial.println(timeStr);

    Serial.print("Unix Timestamp: ");
    Serial.println(now.unixtime());

    Serial.println("========================================");

    return true;
}

// ============================================================
// Check Serial for RTC commands
// ============================================================

void checkSerialForTimeSet()
{
    if (!Serial.available())
        return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0)
        return;

    if (!processTimeSetLine(line))
    {
        Serial.println(
            "Unrecognized command. Use SET MM/DD/YYYY HH:MM:SS AM|PM"
        );
    }
}

// ============================================================
// Set RTC from Serial at startup
// ============================================================

void setRTCFromSerial()
{
    const unsigned long WAIT_MS = 15000UL;

    unsigned long deadline = millis() + WAIT_MS;

    DateTime current = rtc.now();

    char curTimeStr[13];

    formatTime12(
        current,
        curTimeStr,
        sizeof(curTimeStr)
    );

    Serial.println();
    Serial.println("========================================");
    Serial.println("RECEIVER RTC TIME SETTING");

    Serial.print("Current RTC date: ");
    Serial.print(current.month());
    Serial.print("/");
    Serial.print(current.day());
    Serial.print("/");
    Serial.println(current.year());

    Serial.print("Current RTC time: ");
    Serial.println(curTimeStr);

    Serial.println("----------------------------------------");

    Serial.println(
        "SET MM/DD/YYYY HH:MM:SS AM|PM"
    );

    Serial.println("SKIP");

    Serial.println(
        "No input -> Keep current RTC time after 15 seconds."
    );

    Serial.println("========================================");

    while (millis() < deadline)
    {
        if (Serial.available())
        {
            String line = Serial.readStringUntil('\n');
            line.trim();

            if (line.length() == 0)
                continue;

            String cmdUpper = line;
            cmdUpper.toUpperCase();

            if (cmdUpper == "SKIP")
            {
                Serial.println(
                    "SKIP received. Keeping current RTC time."
                );
                return;
            }

            if (processTimeSetLine(line))
                return;

            Serial.println(
                "Invalid command. Use SET ... or SKIP."
            );
        }

        delay(10);
    }

    Serial.println(
        "Timeout reached. Keeping current RTC time."
    );
}

// ============================================================
// Synchronize RTC using the EXISTING Wi-Fi connection
//
// Type SKIP to skip NTP synchronization.
// Otherwise, synchronization starts after 15 seconds.
// This function does not connect or disconnect Wi-Fi.
// ============================================================

void setRTCFromNTP()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("RTC NTP SYNCHRONIZATION");
    Serial.println("========================================");

    // Check existing Wi-Fi connection

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Wi-Fi is not connected.");
        Serial.println("Cannot synchronize RTC using NTP.");
        Serial.println("Keeping current RTC time.");
        Serial.println("========================================");
        return;
    }

    Serial.println("Using existing Wi-Fi connection.");

    Serial.println();
    Serial.println("Type SKIP and press ENTER to skip NTP sync.");
    Serial.println("Otherwise, NTP synchronization will begin");
    Serial.println("automatically after 15 seconds.");

    // Give the user an opportunity to skip

    unsigned long startTime = millis();

    while (millis() - startTime < 15000UL)
    {
        if (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            command.trim();
            command.toUpperCase();

            if (command == "SKIP")
            {
                Serial.println();
                Serial.println("NTP synchronization skipped.");
                Serial.println("Keeping current RTC time.");
                Serial.println("========================================");
                return;
            }

            // Any other input proceeds with NTP synchronization
            break;
        }

        delay(10);
    }

    // Check Wi-Fi again before starting NTP

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Wi-Fi disconnected.");
        Serial.println("NTP synchronization cancelled.");
        Serial.println("Keeping current RTC time.");
        return;
    }

    Serial.println();
    Serial.println("Synchronizing RTC with NTP...");

    configTime(
        NEPAL_GMT_OFFSET_SEC,
        NEPAL_DST_OFFSET_SEC,
        NTP_SERVER
    );

    struct tm timeinfo;

    if (!getLocalTime(&timeinfo, NTP_TIMEOUT_MS))
    {
        Serial.println("NTP synchronization failed.");
        Serial.println("Keeping current RTC time.");
        Serial.println("========================================");
        return;
    }

    // Update DS3231 RTC

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

    formatTime12(
        synced,
        syncStr,
        sizeof(syncStr)
    );

    Serial.println();
    Serial.println("RTC SYNCHRONIZATION SUCCESSFUL");

    Serial.print("Date: ");
    Serial.print(synced.month());
    Serial.print("/");
    Serial.print(synced.day());
    Serial.print("/");
    Serial.println(synced.year());

    Serial.print("Time: ");
    Serial.println(syncStr);

    Serial.print("Unix Timestamp: ");
    Serial.println(synced.unixtime());

    Serial.println("========================================");

    // Wi-Fi remains connected.
}

// ============================================================
// Print current RTC time once per second while enabled
// ============================================================

void printLiveTime()
{
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

// ============================================================
// Configure new Wi-Fi credentials
// Enter SSID, press ENTER.
// Enter password, press ENTER.
// ============================================================

bool configureWiFi()
{
    Serial.println();
    Serial.println("=== Wi-Fi Configuration ===");
    Serial.println("Enter SSID and press ENTER:");

    unsigned long start = millis();

    while (
        !Serial.available() &&
        millis() - start < WIFI_INPUT_TIMEOUT
    )
    {
        delay(100);
    }

    if (!Serial.available())
    {
        Serial.println("SSID input timeout.");
        return false;
    }

    String ssid = Serial.readStringUntil('\n');
    ssid.trim();

    if (ssid.length() == 0)
    {
        Serial.println("Invalid SSID.");
        return false;
    }

    Serial.println("Enter password and press ENTER:");

    start = millis();

    while (
        !Serial.available() &&
        millis() - start < WIFI_INPUT_TIMEOUT
    )
    {
        delay(100);
    }

    if (!Serial.available())
    {
        Serial.println("Password input timeout.");
        return false;
    }

    String password = Serial.readStringUntil('\n');
    password.trim();

    if (password.length() == 0)
    {
        Serial.println("Invalid password.");
        return false;
    }

    preferences.putString("ssid", ssid);
    preferences.putString("password", password);

    savedSSID = ssid;
    savedPassword = password;

    Serial.println("Wi-Fi credentials saved.");

    return connectWiFi();
}

// ============================================================
// Startup Wi-Fi decision
// ============================================================

void startupWiFiSetup()
{
    bool connected = false;

    // First try previously saved credentials

    if (savedSSID.length() > 0)
    {
        Serial.println();
        Serial.println("Checking saved Wi-Fi connection...");

        connected = connectWiFi();
    }

    // If already connected, continue

    if (connected)
    {
        Serial.println("Saved Wi-Fi connection successful.");
        return;
    }

    // No saved credentials or connection failed

    Serial.println();
    Serial.println("========================================");
    Serial.println("WI-FI CONNECTION NOT AVAILABLE");
    Serial.println("========================================");
    Serial.println("Type WIFI to enter new credentials.");
    Serial.println("Type OFF to continue without Wi-Fi.");
    Serial.println("========================================");

    while (true)
    {
        if (!Serial.available())
        {
            delay(10);
            continue;
        }

        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();

        if (command == "OFF")
        {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);

            Serial.println("Continuing without Wi-Fi.");
            return;
        }

        if (command == "WIFI")
        {
            if (configureWiFi())
            {
                Serial.println("Wi-Fi connected successfully.");
                return;
            }

            Serial.println();
            Serial.println("Connection failed.");
            Serial.println("Type WIFI to try again or OFF to continue.");
        }
        else
        {
            Serial.println("Invalid command. Type WIFI or OFF.");
        }
    }
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // --------------------------------------------------------
    // RTC initialization
    // --------------------------------------------------------

    Wire.begin();
    Wire.setClock(100000);

    if (!rtc.begin())
    {
        Serial.println("ERROR: RTC initialization failed.");

        while (true)
            delay(1000);
    }

    Serial.println("RTC initialized successfully.");

    // --------------------------------------------------------
    // Gateway information
    // --------------------------------------------------------

    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 LoRa Temperature Gateway");
    Serial.println("========================================");

    // --------------------------------------------------------
    // LittleFS
    // --------------------------------------------------------

    if (!LittleFS.begin(true))
    {
        Serial.println("ERROR: LittleFS mount failed.");
    }
    else
    {
        Serial.println("LittleFS mounted successfully.");
    }

    // --------------------------------------------------------
    // Preferences
    // --------------------------------------------------------

    preferences.begin("wifi", false);

    savedSSID = preferences.getString("ssid", "");
    savedPassword = preferences.getString("password", "");

    if (savedSSID.length() > 0)
    {
        Serial.println("Saved Wi-Fi credentials found.");
        Serial.print("SSID: ");
        Serial.println(savedSSID);
    }
    else
    {
        Serial.println("No saved Wi-Fi credentials.");
    }

    // --------------------------------------------------------
    // LoRa SPI
    // --------------------------------------------------------

    SPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_SS
    );

    LoRa.setPins(
        LORA_SS,
        LORA_RST,
        LORA_DIO0
    );

    Serial.println("Starting LoRa...");

    if (!LoRa.begin(LORA_FREQ))
    {
        Serial.println("ERROR: LoRa initialization failed.");

        while (true)
            delay(1000);
    }

    // --------------------------------------------------------
    // LoRa configuration
    // --------------------------------------------------------

    LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(16);
    LoRa.setSyncWord(0x12);
    LoRa.enableCrc();

    Serial.println("LoRa initialized.");
    Serial.println("Frequency: 433 MHz");
    Serial.println("SF: 12");
    Serial.println("Bandwidth: 62.5 kHz");
    Serial.println("Coding rate: 4/8");
    Serial.println("CRC: enabled");
    Serial.println("Packet size: 14 bytes");
    Serial.println("TX timestamp: milliseconds");

    // --------------------------------------------------------
    // Serial commands
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Commands:");
    Serial.println("  WIFI   - configure Wi-Fi");
    Serial.println("  ON     - turn Wi-Fi ON / reconnect");
    Serial.println("  OFF    - turn Wi-Fi OFF");
    Serial.println("  STATUS - show Wi-Fi status");
    Serial.println("  SET    - update RTC time");
    Serial.println("  TIME   - print live time every second");
    Serial.println("  STOP   - stop live time print");
    Serial.println();

    // --------------------------------------------------------
    // Startup Wi-Fi check and user decision
    // --------------------------------------------------------

    startupWiFiSetup();

    // --------------------------------------------------------
    // Synchronize RTC using existing Wi-Fi connection
    // --------------------------------------------------------

    setRTCFromNTP();

    // --------------------------------------------------------
    // Allow manual RTC setting
    // --------------------------------------------------------

    setRTCFromSerial();

    // --------------------------------------------------------
    // Gateway ready
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Gateway ready.");
    Serial.println("Waiting for 14-byte LoRa packets...");
    Serial.println();
}

// ============================================================
// Main loop
// ============================================================

void loop()
{
    processSerialCommands();

    receiveLoRaPacket();

    printLiveTime();

    delay(10);
}

// ============================================================
// Connect Wi-Fi
// ============================================================

bool connectWiFi()
{
    if (savedSSID.length() == 0)
    {
        Serial.println("No saved Wi-Fi credentials.");
        return false;
    }

    Serial.println();
    Serial.println("Connecting to Wi-Fi...");
    Serial.print("SSID: ");
    Serial.println(savedSSID);

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        savedSSID.c_str(),
        savedPassword.c_str()
    );

    unsigned long startTime = millis();

    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - startTime < 20000
    )
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("Wi-Fi connected.");

        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        Serial.println("Checking LittleFS queue...");

        sendQueuedPackets();

        return true;
    }

    Serial.println("Wi-Fi connection failed.");
    return false;
}

// ============================================================
// Serial commands
// ============================================================

void processSerialCommands()
{
    if (!Serial.available())
        return;

    String command = Serial.readStringUntil('\n');

    command.trim();

    // Do not uppercase the full command because SET parsing
    // uses the original command text. AM/PM is handled case-insensitively.

    String commandUpper = command;
    commandUpper.toUpperCase();

    // --------------------------------------------------------
    // TIME / STOP
    // --------------------------------------------------------

    if (commandUpper == "TIME")
    {
        livePrintEnabled = true;
        lastPrintedSecond = -1;

        Serial.println("Live time print STARTED. Send STOP to stop.");
        return;
    }

    if (commandUpper == "STOP")
    {
        livePrintEnabled = false;

        Serial.println("Live time print STOPPED.");
        return;
    }

    // --------------------------------------------------------
    // RTC SET
    // --------------------------------------------------------

    if (commandUpper.startsWith("SET"))
    {
        processTimeSetLine(command);
        return;
    }

    // --------------------------------------------------------
    // WIFI
    // --------------------------------------------------------

    if (commandUpper == "WIFI")
    {
        configureWiFi();
        return;
    }

    // --------------------------------------------------------
    // ON
    // --------------------------------------------------------

    if (commandUpper == "ON")
    {
        Serial.println("Turning Wi-Fi ON...");
        connectWiFi();
        return;
    }

    // --------------------------------------------------------
    // OFF
    // --------------------------------------------------------

    if (commandUpper == "OFF")
    {
        Serial.println("Turning Wi-Fi OFF...");

        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);

        Serial.println("Wi-Fi turned OFF.");
        return;
    }

    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    if (commandUpper == "STATUS")
    {
        printWiFiStatus();
        return;
    }

    Serial.println("Unknown command.");
}

// ============================================================
// Wi-Fi status
// ============================================================

void printWiFiStatus()
{
    Serial.println();
    Serial.println("=== Wi-Fi Status ===");

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("Status: CONNECTED");

        Serial.print("SSID: ");
        Serial.println(WiFi.SSID());

        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        Serial.print("RSSI: ");
        Serial.println(WiFi.RSSI());
    }
    else
    {
        Serial.println("Status: DISCONNECTED");
    }

    Serial.println();
}

// ============================================================
// Receive LoRa packet - 14 BYTES
// ============================================================

void receiveLoRaPacket()
{
    int packetSize = LoRa.parsePacket();

    if (packetSize == 0)
        return;

    int receivedLength = 0;

    // --------------------------------------------------------
    // Read binary packet
    // --------------------------------------------------------

    while (
        LoRa.available() &&
        receivedLength < PACKET_SIZE
    )
    {
        receivedData[receivedLength++] =
            (uint8_t)LoRa.read();
    }

    // Clear remaining bytes

    while (LoRa.available())
    {
        LoRa.read();
    }

    // --------------------------------------------------------
    // Validate packet size
    // --------------------------------------------------------

    if (
        packetSize != PACKET_SIZE ||
        receivedLength != PACKET_SIZE
    )
    {
        Serial.println("Invalid packet size!");

        Serial.print("Expected: ");
        Serial.println(PACKET_SIZE);

        Serial.print("Received: ");
        Serial.println(packetSize);

        return;
    }

    // --------------------------------------------------------
    // Decode packet
    // --------------------------------------------------------

    uint8_t nodeID = receivedData[0];

    uint8_t packetID = receivedData[1];

    // Temperature: signed int16, little-endian

    int16_t tempRaw =
        (int16_t)(
            (uint16_t)receivedData[2] |
            ((uint16_t)receivedData[3] << 8)
        );

    // Humidity: unsigned uint16, little-endian

    uint16_t humidityRaw =
        (uint16_t)receivedData[4] |
        ((uint16_t)receivedData[5] << 8);

    // --------------------------------------------------------
    // Decode 64-bit TX timestamp in milliseconds
    // Bytes 6-13, little-endian
    // --------------------------------------------------------

    uint64_t txTimestampMs = 0;

    for (int i = 0; i < 8; i++)
    {
        txTimestampMs |=
            ((uint64_t)receivedData[6 + i]) << (8 * i);
    }

    // Convert sensor values

    float temperature = tempRaw / 100.0f;

    float humidity = humidityRaw / 100.0f;

    // --------------------------------------------------------
    // Capture RX timestamp in milliseconds
    // --------------------------------------------------------

    DateTime rxTime = rtc.now();

    uint64_t rxTimestampMs =
        ((uint64_t)rxTime.unixtime() * 1000ULL) +
        (millis() % 1000UL);

    // --------------------------------------------------------
    // Calculate delay in milliseconds
    // --------------------------------------------------------

    int64_t delayMilliseconds =
        (int64_t)rxTimestampMs -
        (int64_t)txTimestampMs;

    float delaySeconds =
        delayMilliseconds / 1000.0f;

    // --------------------------------------------------------
    // Format TX and RX timestamps
    // --------------------------------------------------------

    char txTimeString[32];

    char rxTimeString[32];

    formatTimestampMs(
        txTimestampMs,
        txTimeString,
        sizeof(txTimeString)
    );

    formatTimestampMs(
        rxTimestampMs,
        rxTimeString,
        sizeof(rxTimeString)
    );

    // --------------------------------------------------------
    // Signal information
    // --------------------------------------------------------

    int rssi = LoRa.packetRssi();

    float snr = LoRa.packetSnr();

    // --------------------------------------------------------
    // Display received information
    // --------------------------------------------------------

    Serial.println();
    Serial.println("========================================");
    Serial.println("LoRa BINARY PACKET RECEIVED");
    Serial.println("========================================");

    Serial.print("Packet size: ");
    Serial.print(packetSize);
    Serial.println(" bytes");

    Serial.print("Node ID: ");
    Serial.println(nodeID);

    Serial.print("Packet ID: ");
    Serial.println(packetID);

    Serial.print("Temperature: ");

    if (tempRaw == INT16_MIN)
    {
        Serial.println("Sensor Error");
    }
    else
    {
        Serial.print(temperature, 2);
        Serial.println(" °C");
    }

    Serial.print("Humidity: ");

    if (humidityRaw == UINT16_MAX)
    {
        Serial.println("Sensor Error");
    }
    else
    {
        Serial.print(humidity, 2);
        Serial.println(" %");
    }

    Serial.print("RX Timestamp (ms): ");
    Serial.println((unsigned long long)rxTimestampMs);

    Serial.print("TX Date/Time: ");
    Serial.println(txTimeString);

    Serial.print("RX Date/Time: ");
    Serial.println(rxTimeString);

    Serial.print("Delay: ");
    Serial.print((long long)delayMilliseconds);
    Serial.println(" milliseconds");

    Serial.print("Delay: ");
    Serial.print(delaySeconds, 3);
    Serial.println(" seconds");

    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");

    Serial.print("SNR: ");
    Serial.print(snr);
    Serial.println(" dB");

    // --------------------------------------------------------
    // Raw HEX
    // --------------------------------------------------------

    Serial.print("Raw Data: ");

    for (int i = 0; i < PACKET_SIZE; i++)
    {
        if (receivedData[i] < 0x10)
            Serial.print("0");

        Serial.print(receivedData[i], HEX);
        Serial.print(" ");
    }

    Serial.println();

    Serial.println("========================================");

    // --------------------------------------------------------
    // Convert binary packet to internal JSON
    // --------------------------------------------------------

    JsonDocument packetDoc;

    String deviceID = "TX" + String(nodeID);

    packetDoc["n"] = deviceID;

    packetDoc["id"] = packetID;

    // Sensor error sentinel

    packetDoc["t"] =
        (tempRaw == INT16_MIN)
        ? -999.0f
        : temperature;

    packetDoc["h"] =
        (humidityRaw == UINT16_MAX)
        ? -999.0f
        : humidity;

    // TX timestamp

    packetDoc["tm"] = txTimeString;

    // RX timestamp

    packetDoc["rx"] = rxTimeString;

    // Delay in milliseconds

    packetDoc["delay_ms"] =
        delayMilliseconds;

    // Delay in seconds

    packetDoc["delay"] =
        delaySeconds;

    // --------------------------------------------------------
    // Serialize packet
    // --------------------------------------------------------

    String packet = "";

    serializeJson(packetDoc, packet);

    Serial.println("Converted packet for API:");
    Serial.println(packet);

    // --------------------------------------------------------
    // Send to API or save to queue
    // --------------------------------------------------------

    if (WiFi.status() == WL_CONNECTED)
    {
        bool success = sendToAPI(
            packet,
            rssi,
            snr
        );

        if (!success)
        {
            Serial.println("API upload failed.");
            Serial.println("Saving packet to queue...");

            saveToQueue(
                packet,
                rssi,
                snr
            );
        }
    }
    else
    {
        Serial.println("Wi-Fi is OFFLINE.");
        Serial.println("Saving packet to queue...");

        saveToQueue(
            packet,
            rssi,
            snr
        );
    }
}

// ============================================================
// Check whether packet is already API format
// ============================================================

bool isAPIFormat(const String &packet)
{
    JsonDocument doc;

    DeserializationError error =
        deserializeJson(doc, packet);

    if (error)
        return false;

    return doc["packet_id"].is<int>();
}

// ============================================================
// Convert internal LoRa JSON to API JSON
// ============================================================

bool convertLoRaToAPI(
    const String &input,
    String &output,
    int rssi,
    float snr)
{
    JsonDocument inputDoc;

    DeserializationError error =
        deserializeJson(inputDoc, input);

    if (error)
    {
        Serial.print("JSON conversion error: ");
        Serial.println(error.c_str());

        return false;
    }

    // --------------------------------------------------------
    // Verify required fields
    // --------------------------------------------------------

    if (
        !inputDoc["n"].is<const char *>() ||
        !inputDoc["id"].is<int>() ||
        !inputDoc["t"].is<float>() ||
        !inputDoc["h"].is<float>() ||
        !inputDoc["tm"].is<const char *>()
    )
    {
        Serial.println("ERROR: Invalid LoRa packet format.");
        return false;
    }

    // --------------------------------------------------------
    // Build API document
    // --------------------------------------------------------

    JsonDocument apiDoc;

    apiDoc["packet_id"] =
        inputDoc["id"].as<int>();

    apiDoc["device_id"] =
        inputDoc["n"].as<const char *>();

    apiDoc["timestamp"] =
        inputDoc["tm"].as<const char *>();

    // RX timestamp

    if (inputDoc["rx"].is<const char *>())
    {
        apiDoc["rx_timestamp"] =
            inputDoc["rx"].as<const char *>();
    }

    // Delay in milliseconds

    if (inputDoc["delay_ms"].is<int64_t>())
    {
        apiDoc["delay_milliseconds"] =
            inputDoc["delay_ms"].as<int64_t>();
    }

    apiDoc["temperature"] =
        inputDoc["t"].as<float>();

    apiDoc["humidity"] =
        inputDoc["h"].as<float>();

    apiDoc["rssi"] = rssi;

    apiDoc["snr"] = snr;

    output = "";

    serializeJson(apiDoc, output);

    return true;
}

// ============================================================
// Convert queued packet
// ============================================================

bool convertQueuedPacket(
    const String &input,
    String &output)
{
    if (isAPIFormat(input))
    {
        output = input;
        return true;
    }

    return convertLoRaToAPI(
        input,
        output,
        -1,
        -1.0
    );
}

// ============================================================
// Send packet to API
// ============================================================

bool sendToAPI(
    const String &packet,
    int rssi,
    float snr)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(
            "Cannot upload: Wi-Fi disconnected."
        );

        return false;
    }

    String apiPacket;

    if (isAPIFormat(packet))
    {
        apiPacket = packet;
    }
    else
    {
        if (
            !convertLoRaToAPI(
                packet,
                apiPacket,
                rssi,
                snr
            )
        )
        {
            Serial.println(
                "ERROR: Could not convert packet."
            );

            return false;
        }
    }

    Serial.println("Sending to API:");
    Serial.println(apiPacket);

    HTTPClient http;

    http.begin(API_URL);

    http.addHeader(
        "Content-Type",
        "application/json"
    );

    int httpCode = http.POST(apiPacket);

    Serial.print("HTTP response: ");
    Serial.println(httpCode);

    if (httpCode > 0)
    {
        String response = http.getString();

        Serial.print("API response: ");
        Serial.println(response);
    }
    else
    {
        Serial.print("HTTP error: ");

        Serial.println(
            http.errorToString(httpCode)
        );
    }

    http.end();

    if (httpCode >= 200 && httpCode < 300)
    {
        Serial.println("API upload successful.");
        return true;
    }

    Serial.println("API upload failed.");

    return false;
}

// ============================================================
// Save packet to LittleFS queue
// ============================================================

bool saveToQueue(
    const String &packet,
    int rssi,
    float snr)
{
    String apiPacket;

    if (isAPIFormat(packet))
    {
        apiPacket = packet;
    }
    else
    {
        if (
            !convertLoRaToAPI(
                packet,
                apiPacket,
                rssi,
                snr
            )
        )
        {
            Serial.println(
                "ERROR: Could not convert packet for queue."
            );

            return false;
        }
    }

    File file = LittleFS.open(
        QUEUE_FILE,
        "a"
    );

    if (!file)
    {
        Serial.println(
            "ERROR: Could not open queue file."
        );

        return false;
    }

    file.println(apiPacket);

    file.close();

    Serial.println("Packet saved to queue:");
    Serial.println(apiPacket);

    return true;
}

// ============================================================
// Check whether queue file exists
// ============================================================

bool queueFileExists()
{
    File root = LittleFS.open("/");

    if (!root)
    {
        Serial.println(
            "ERROR: Could not open LittleFS root."
        );

        return false;
    }

    if (!root.isDirectory())
    {
        root.close();
        return false;
    }

    File entry = root.openNextFile();

    while (entry)
    {
        String name = entry.name();

        entry.close();

        if (
            name == QUEUE_FILE ||
            name == "queue.txt"
        )
        {
            root.close();
            return true;
        }

        entry = root.openNextFile();
    }

    root.close();

    return false;
}

// ============================================================
// Send queued packets
// ============================================================

bool sendQueuedPackets()
{
    if (!queueFileExists())
    {
        Serial.println("No queued packets.");
        return true;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(
            "Cannot process queue: Wi-Fi disconnected."
        );

        return false;
    }

    File file = LittleFS.open(
        QUEUE_FILE,
        "r"
    );

    if (!file)
    {
        Serial.println(
            "ERROR: Could not open queue file."
        );

        return false;
    }

    Serial.println();
    Serial.println("========================================");
    Serial.println("Sending queued packets...");
    Serial.println("========================================");

    bool allSuccessful = true;

    while (file.available())
    {
        String line = file.readStringUntil('\n');

        line.trim();

        if (line.length() == 0)
            continue;

        Serial.println();
        Serial.println("Queued packet:");
        Serial.println(line);

        String apiPacket;

        if (!convertQueuedPacket(line, apiPacket))
        {
            Serial.println(
                "ERROR: Could not process queued packet."
            );

            allSuccessful = false;
            break;
        }

        Serial.println("Sending queued API packet:");
        Serial.println(apiPacket);

        HTTPClient http;

        http.begin(API_URL);

        http.addHeader(
            "Content-Type",
            "application/json"
        );

        int httpCode = http.POST(apiPacket);

        Serial.print("HTTP response: ");
        Serial.println(httpCode);

        if (httpCode > 0)
        {
            String response = http.getString();

            Serial.print("API response: ");
            Serial.println(response);
        }
        else
        {
            Serial.print("HTTP error: ");

            Serial.println(
                http.errorToString(httpCode)
            );
        }

        http.end();

        if (httpCode >= 200 && httpCode < 300)
        {
            Serial.println(
                "Queued packet uploaded successfully."
            );
        }
        else
        {
            Serial.println(
                "Queued packet upload FAILED."
            );

            allSuccessful = false;
            break;
        }

        delay(100);
    }

    file.close();

    // Delete queue only if every packet was uploaded

    if (allSuccessful)
    {
        Serial.println();
        Serial.println(
            "All queued packets uploaded successfully."
        );

        if (LittleFS.remove(QUEUE_FILE))
        {
            Serial.println("LittleFS queue cleared.");
            Serial.println("queue.txt deleted.");
        }
        else
        {
            Serial.println(
                "WARNING: Could not delete queue.txt."
            );
        }

        return true;
    }

    Serial.println();
    Serial.println(
        "Some queued packets were not uploaded."
    );

    Serial.println(
        "Queue retained for next Wi-Fi connection."
    );

    return false;
}
