#include <Arduino.h>
#include <ArduinoNvs.h>
#include <Arduino_JSON.h>
#include <ESP32Time.h>
#include <HTTPClient.h>
#include <WiFi.h>
//#define DEBUG

#define CLK 26
#define DAT 25
#define CLOCK_DELAY 4

#define BUF_LEN 64
char buf[BUF_LEN];
int bufLen = 0;

bool displayDots = false;

uint8_t seg[] = { 
  0b01111110, // 0
  0b01001000, // 1
  0b00111101, // 2
  0b01101101, // 3
  0b01001011, // 4
  0b01100111, // 5
  0b01110111, // 6
  0b01001100, // 7
  0b01111111, // 8
  0b01101111, // 9
                  
  0b11100111, // 0
  0b00100001, // 1
  0b11001011, // 2
  0b01101011, // 3
  0b00101101, // 4
  0b01101110, // 5
  0b11101110, // 6
  0b00100011, // 7
  0b11101111, // 8
  0b01101111  // 9
  };

String timezone;
String ssid;
String password;

ESP32Time rtc;

unsigned long lastTimeFetch = 0;
unsigned long timeFetchInterval = 30 * 60 * 1000; // 30 min

unsigned long lastTimeDisplay = 0;
unsigned long timeDisplayInterval = 500; // 0.5 s

uint8_t display_data[5];

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
    String serverPath = "http://worldtimeapi.org/api/timezone/" + timezone;
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
  timezone = NVS.getString("timezone");
  ssid = NVS.getString("ssid");
  password = NVS.getString("password");

  WiFi.onEvent(wifi_evt_con, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(wifi_evt_ip, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(wifi_evt_dis, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.disconnect(true);


  WiFi.begin(ssid, password);

  pinMode(CLK, OUTPUT);
  pinMode(DAT, OUTPUT);
  digitalWrite(CLK, LOW);
  digitalWrite(DAT, LOW);
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
        Serial.print(timezone);
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
        timezone = String(cmd.substring(3));
        timezone.trim();
        Serial.print("Timezone: ");
        Serial.print("\"");
        Serial.print(timezone);
        Serial.println("\"");
        NVS.setString("timezone", timezone);

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

void loadData(uint8_t *data) {
  
  digitalWrite(CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(CLK, LOW);
  delayMicroseconds(CLOCK_DELAY / 2);

  digitalWrite(DAT, HIGH);

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(CLK, LOW);
  delayMicroseconds(CLOCK_DELAY / 2);

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(CLK, LOW);
  delayMicroseconds(CLOCK_DELAY / 2);

  
  for (int i = 0; i < 34; ++i) {
    bool high = (data[i / 8] & (1 << (7 - (i % 8))));
    
    if (high) digitalWrite(DAT, HIGH);
    else digitalWrite(DAT, LOW);

    delayMicroseconds(CLOCK_DELAY / 2);

    digitalWrite(CLK, HIGH);
    delayMicroseconds(CLOCK_DELAY);
    digitalWrite(CLK, LOW);
    delayMicroseconds(CLOCK_DELAY / 2);
  }

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(DAT, LOW);
}

void displayTime(bool dots) {
  uint8_t hour = rtc.getHour(true);
  uint8_t minute = rtc.getMinute();

  uint8_t hour_first_digit = hour / 10;
  uint8_t hour_second_digit = hour % 10;

  uint8_t minute_first_digit = minute / 10;
  uint8_t minute_second_digit = minute % 10;

  uint8_t seg1 = seg[hour_first_digit];
  uint8_t seg2 = seg[hour_second_digit];
  uint8_t seg3 = seg[minute_first_digit + 10];
  uint8_t seg4 = seg[minute_second_digit + 10];

  display_data[0] = seg1;
  display_data[1] = seg2;
  if (dots) display_data[2] = 0b11000000;
  else display_data[2] = 0;
  display_data[2] |= (0b00111111 & (seg3 >> 2));
  display_data[3] = (((0b00000011 & seg3) << 6));
  display_data[3] |= (0b00111111 & (seg4 >> 2));
  display_data[4] = (((0b00000011 & seg4) << 6));
  
  loadData(display_data);
}

void loop() {

  if ((millis() - lastTimeFetch) > timeFetchInterval && hasConnection) {
    requestTime();
    lastTimeFetch = millis();
  }

  if ((millis() - lastTimeDisplay) > timeDisplayInterval) {
    lastTimeDisplay = millis();
    displayTime(displayDots);
    displayDots = !displayDots;
  }

  processSerial();
}
