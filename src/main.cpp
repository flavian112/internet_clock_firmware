#include <Arduino.h>
#include <ArduinoNvs.h>
#include <Arduino_JSON.h>
#include <ESP32Time.h>
#include <HTTPClient.h>
#include <WiFi.h>

#define PIN_CLK 26
#define PIN_DAT 25
#define CLOCK_DELAY 4 // microseconds, frequenzy is roughly 1 / (2 * CLOCK_DELAY)

#define SERIAL_BUF_LEN 64
char serial_buf[SERIAL_BUF_LEN];
int serial_buf_len = 0;

bool display_dots = false;

const uint8_t seg[] = { // table to convert digit to segments to be enabled
  // for first and second digit
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
  // for third and forth digit
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

const String time_api_server = "http://worldtimeapi.org/api/timezone/";

ESP32Time rtc;

unsigned long last_time_fetch = 0;
const unsigned long time_fetch_interval = 30 * 60 * 1000; // 30 min

unsigned long last_time_display = 0;
const unsigned long time_display_interval = 500; // 0.5 s

uint8_t display_data[5];

bool has_internet_connection = false;

String json_buf;

String httpGETRequest(const char *serverName) {
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverName);
  int http_response_code = http.GET();
  String payload = "{}";
  if (http_response_code > 0) payload = http.getString();
  http.end();
  return payload;
}

void request_time() {
  if (WiFi.status() != WL_CONNECTED) return;
  String server_path = time_api_server + timezone;
  json_buf = httpGETRequest(server_path.c_str());
  JSONVar result = JSON.parse(json_buf);
  if (JSON.typeof(result) == "undefined") return;
  time_t currentTime = result["unixtime"];
  long offset = result["raw_offset"];
  rtc.setTime(currentTime);
  rtc.offset = offset;
}

void wifi_evt_ip(WiFiEvent_t event, WiFiEventInfo_t info) {
  has_internet_connection = true;
  request_time();
}

void wifi_evt_dis(WiFiEvent_t event, WiFiEventInfo_t info) {
  has_internet_connection = false;
  WiFi.begin(ssid, password);
}

void setup() {
  Serial.begin(9600);

  NVS.begin();
  timezone = NVS.getString("timezone");
  ssid = NVS.getString("ssid");
  password = NVS.getString("password");

  WiFi.onEvent(wifi_evt_ip, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(wifi_evt_dis, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);

  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DAT, OUTPUT);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_DAT, LOW);
}

void process_serial() {
  while(Serial.available()) {
    if (serial_buf_len >= SERIAL_BUF_LEN - 1) {
      serial_buf_len = 0;
    }
    char input = Serial.read();
    Serial.write(input);
    serial_buf[serial_buf_len++] = input;
    if (input == '\n') {
      String cmd = String(serial_buf, serial_buf_len);
      serial_buf_len = 0;

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

void update_display_data(bool dots) {
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

  display_data[0] = seg2;
  display_data[1] = seg1;
  if (dots) display_data[2] = 0b11000000;
  else display_data[2] = 0;
  display_data[2] |= (0b00111111 & (seg3 >> 2));
  display_data[3] = (((0b00000011 & seg3) << 6));
  display_data[3] |= (0b00111111 & (seg4 >> 2));
  display_data[4] = (((0b00000011 & seg4) << 6));
}

void write_display_data() {
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(CLOCK_DELAY / 2);

  digitalWrite(PIN_DAT, HIGH);

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(CLOCK_DELAY / 2);

  digitalWrite(PIN_DAT, LOW);

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(CLOCK_DELAY / 2);

  for (int i = 0; i < 34; ++i) {
    bool high = (display_data[i / 8] & (1 << (7 - (i % 8))));
    
    if (high) digitalWrite(PIN_DAT, HIGH);
    else digitalWrite(PIN_DAT, LOW);

    delayMicroseconds(CLOCK_DELAY / 2);

    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(CLOCK_DELAY);
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(CLOCK_DELAY / 2);
  }

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(PIN_DAT, LOW);

  delayMicroseconds(CLOCK_DELAY / 2);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(PIN_CLK, LOW);
}

void loop() {

  if ((millis() - last_time_fetch) > time_fetch_interval && has_internet_connection) {
    request_time();
    last_time_fetch = millis();
  }

  if ((millis() - last_time_display) > time_display_interval) {
    last_time_display = millis();
    update_display_data(display_dots);
    display_dots = !display_dots;
    write_display_data();
  }

  process_serial();
}