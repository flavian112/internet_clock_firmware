#include <Arduino.h>
#include <ArduinoNvs.h>
#include <Arduino_JSON.h>
#include <ESP32Time.h>
#include <HTTPClient.h>
#include <WiFi.h>
//#define DEBUG

#define BUF_LEN 64
char buf[BUF_LEN];
int bufLen = 0;

String timeZone;
String ssid;
String password;

ESP32Time rtc;

unsigned long lastUpdate = 0;
unsigned long updateInterval = 30 * 60 * 1000; // 30 min

bool hasConnection = false;

String jsonBuffer;

String httpGETRequest(const char *serverName) {
  WiFiClient client;
  HTTPClient http;

  http.begin(client, serverName);

  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    payload = http.getString();
  }
  http.end();

  return payload;
}

void requestTime() {
  if(WiFi.status()== WL_CONNECTED) {
    String serverPath = "http://worldtimeapi.org/api/timezone/" + timeZone;
    jsonBuffer = httpGETRequest(serverPath.c_str());
    JSONVar myObject = JSON.parse(jsonBuffer);
    if (JSON.typeof(myObject) == "undefined") {
      return;
    }
    time_t currentTime = myObject["unixtime"];
    long offset = myObject["raw_offset"];
    rtc.setTime(currentTime);
    rtc.offset = offset;
  }
}

void wifi_evt_con(WiFiEvent_t event, WiFiEventInfo_t info) {
#ifdef DEBUG
  Serial.print("wifi: connected, ssid: ");
  Serial.println(ssid);
#endif
}

void wifi_evt_ip(WiFiEvent_t event, WiFiEventInfo_t info) {
#ifdef DEBUG
  Serial.print("wifi: ip: ");
  Serial.println(WiFi.localIP());
#endif
  hasConnection = true;
  requestTime();
}

void wifi_evt_dis(WiFiEvent_t event, WiFiEventInfo_t info) {
#ifdef DEBUG
  Serial.print("wifi: disconnected, reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("wifi: reconnecting");
#endif
  hasConnection = false;
  WiFi.begin(ssid, password);
}

void setup() {
  Serial.begin(9600);

  NVS.begin();
  timeZone = NVS.getString("timezone");
  ssid = NVS.getString("ssid");
  password = NVS.getString("password");

  WiFi.onEvent(wifi_evt_con, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(wifi_evt_ip, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(wifi_evt_dis, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.disconnect(true);


  WiFi.begin(ssid, password);
}

void processSerial() {
  while(Serial.available()) {
    if (bufLen >= BUF_LEN - 1) {
      bufLen = 0;
    }
    char input = Serial.read();
    Serial.write(input);
    buf[bufLen++] = input;
    if (input == '\n') {
      String cmd = String(buf, bufLen);
      bufLen = 0;

      if (cmd.startsWith("pt")) {
        Serial.println(rtc.getDateTime());

      } else if (cmd.startsWith("ps")) {
        Serial.print("SSID: ");
        Serial.print("\"");
        Serial.print(ssid);
        Serial.println("\"");

      } else if (cmd.startsWith("pz")) {
        Serial.print("Timezone: ");
        Serial.print("\"");
        Serial.print(timeZone);
        Serial.println("\"");

      } else if (cmd.startsWith("pp")) {
        Serial.print("Password: ");
        Serial.print("\"");
        Serial.print(password);
        Serial.println("\"");

      } else if (cmd.startsWith("ss ")) {
        ssid = String(cmd.substring(3));
        ssid.trim();
        Serial.print("SSID: ");
        Serial.print("\"");
        Serial.print(ssid);
        Serial.println("\"");
        NVS.setString("ssid", ssid);

      } else if (cmd.startsWith("sz ")) {
        timeZone = String(cmd.substring(3));
        timeZone.trim();
        Serial.print("Timezone: ");
        Serial.print("\"");
        Serial.print(timeZone);
        Serial.println("\"");
        NVS.setString("timezone", timeZone);

      } else if (cmd.startsWith("sp ")) {
        password = String(cmd.substring(3));
        password.trim();
        Serial.print("Password: ");
        Serial.print("\"");
        Serial.print(password);
        Serial.println("\"");
        NVS.setString("password", password);
      }
    }
  }
}



void loop() {

  if ((millis() - lastUpdate) > updateInterval && hasConnection) {
    requestTime();
    lastUpdate = millis();
  }

  processSerial();
}
