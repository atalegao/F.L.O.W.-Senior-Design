// ============================================================
//  Flood Monitor — ESP8266 WiFi Module
//  Receives FLOOD alert from STM32 receiver node over UART,
//  then sends a Telegram message to the user's phone.
//
//  WIRING (ESP8266 <-> STM32):
//    D5 (GPIO14) = RX  <-  STM32 USART2 TX
//    D6 (GPIO12) = TX  ->  STM32 USART2 RX
//    GND                   GND (shared)
//
//  LIBRARIES NEEDED (install via Arduino IDE Library Manager):
//    - ESP8266WiFi        (comes with ESP8266 board package)
//    - ESPAsyncTCP        (by dvarrel)
//    - ESPAsyncWebServer  (by lacamera)
//    - ESP8266mDNS        (comes with ESP8266 board package)
//    - ArduinoJson        (by bblanchon)
// ============================================================

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <SoftwareSerial.h>
#include "time.h"

// ------------------------------------------------------------
// WiFi credentials — update to your network
// ------------------------------------------------------------
const char* ssid     = "evergreen";
const char* password = "password!";

// ------------------------------------------------------------
// Telegram bot credentials
// Create a bot via @BotFather on Telegram, paste token below.
// Get your chat ID by messaging @userinfobot on Telegram.
// ------------------------------------------------------------
const char* botToken    = "8697804056:AAHT2ysDRShe_MU7thxa4_Bdix2LqLz2-6A";
const char* chatID      = "8502850102";

// ------------------------------------------------------------
// NTP time server (for timestamp in alert messages)
// ------------------------------------------------------------
const char* ntpServer       = "pool.ntp.org";
const long  gmtOffset_sec   = -18000; // UTC-5 (EST) — adjust for your timezone
const int   daylightOffset_sec = 3600;

// ------------------------------------------------------------
// SoftwareSerial — communication with STM32
// D5 = RX (receives from STM32 TX)
// D6 = TX (sends to STM32 RX) — reserved for future use
// ------------------------------------------------------------
SoftwareSerial stmSerial(14, 12); // RX=D5(GPIO14), TX=D6(GPIO12)

// ------------------------------------------------------------
// sendTelegramMessage()
// Sends a plain text message to the configured Telegram chat.
// Spaces in the message should be replaced with '+' before
// calling this function.
// ------------------------------------------------------------
void sendTelegramMessage(String message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Telegram] WiFi not connected, skipping.");
    return;
  }

  BearSSL::WiFiClientSecure client;
  client.setInsecure(); // OK for prototyping — disables cert check

  HTTPClient https;

  String url = "https://api.telegram.org/bot" + String(botToken) +
               "/sendMessage?chat_id=" + String(chatID) +
               "&text=" + message;

  if (https.begin(client, url)) {
    int httpCode = https.GET();
    if (httpCode > 0) {
      Serial.printf("[Telegram] Sent OK, HTTP code: %d\n", httpCode);
    } else {
      Serial.printf("[Telegram] Failed: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  } else {
    Serial.println("[Telegram] Unable to connect to API.");
  }
}

void setup() {
  // put your setup code here, to run once:
  // 1. Debug serial (USB → Serial Monitor)
  Serial.begin(115200);
  Serial.println("\nFlood Monitor ESP8266 booting...");

  // 2. SoftwareSerial to STM32
  stmSerial.begin(9600);
  Serial.println("STM32 serial started.");

  // 3. Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  // 4. Sync time over NTP (needed for timestamps in alerts)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.print("Syncing time");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nTime synced.");

  // 5. Send a startup confirmation to Telegram
  sendTelegramMessage("Flood+Monitor+Online+and+watching.");
  Serial.println("Startup Telegram message sent.");
}


void loop() {
  // put your main code here, to run repeatedly:
  // Listen for messages from STM32 over SoftwareSerial
  if (stmSerial.available()) {
    String msg = stmSerial.readStringUntil('\n');
    msg.trim();
    Serial.println("RX from STM32: " + msg);

    // Expected format from STM32: "FLOOD:<nodeId>:<distance_mm>"
    // e.g. "FLOOD:1:142"
    if (msg.startsWith("FLOOD:")) {
      int firstColon  = msg.indexOf(':');
      int secondColon = msg.indexOf(':', firstColon + 1);

      if (firstColon != -1 && secondColon != -1) {
        String nodeId   = msg.substring(firstColon + 1, secondColon);
        String distance = msg.substring(secondColon + 1);

        // Get current time for the alert message
        struct tm timeinfo;
        String timeStr = "unknown+time";
        if (getLocalTime(&timeinfo)) {
          char timeBuf[20];
          strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &timeinfo);
          timeStr = String(timeBuf);
          timeStr.replace(":", "%3A"); // URL-encode colons for Telegram URL
        }

        // Build and send Telegram alert
        String telegramMsg = "FLOOD+ALERT%0A"
                             "Sensor+Node:+" + nodeId + "%0A"
                             "Water+Level:+" + distance + "+mm%0A"
                             "Time:+" + timeStr + "%0A"
                             "Potential+flooding+detected!";

        sendTelegramMessage(telegramMsg);
        Serial.println("Flood alert sent for node " + nodeId +
                       " (" + distance + " mm)");
      }
    }
  }
  // static uint32_t lastTest = 0;
  // if (millis() - lastTest >= 3000) {
  //   Serial.println("Sending test to STM32...");
  //   stmSerial.println("HELLO");
  //   lastTest = millis();
  // }

  // if (stmSerial.available()) {
  //   String msg = stmSerial.readStringUntil('\n');
  //   msg.trim();
  //   Serial.println("RX from STM32: " + msg);
  // }
}

