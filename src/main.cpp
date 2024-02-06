#include <Arduino.h>
#include <ArduinoNvs.h>
#include <Arduino_JSON.h>
#include <ESP32Time.h>
#include <HTTPClient.h>
#include <WiFi.h>

#define PIN_CLK 26
#define PIN_DAT 25
#define CLOCK_DELAY 2 // microseconds, frequenzy is roughly 1 / (4 * CLOCK_DELAY)

#define SERIAL_BUF_LEN 64

char serial_buf[SERIAL_BUF_LEN];
int serial_buf_len = 0;

uint8_t display_data[5];
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

bool has_internet_connection = false;
String json_buf;

unsigned long last_time_fetch = 0;
const unsigned long time_fetch_interval = 30 * 60 * 1000; // 30 min
unsigned long last_time_display = 0;
const unsigned long time_display_interval = 1000; // 0.5 s

String perform_http_get_request(const char *serverName) {
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
  json_buf = perform_http_get_request(server_path.c_str());
  JSONVar result = JSON.parse(json_buf);
  if (JSON.typeof(result) == "undefined") return;
  if (!result.hasOwnProperty("unixtime")) return;
  if (!result.hasOwnProperty("raw_offset")) return;
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

void print_string(String desc, String str) {
  Serial.print(desc);
  Serial.print(": ");
  Serial.print("\"");
  Serial.print(str);
  Serial.println("\"");
}

void process_serial() {
  while(Serial.available()) {
    if (serial_buf_len >= SERIAL_BUF_LEN - 1) serial_buf_len = 0;
    char input = Serial.read();
    Serial.write(input);
    serial_buf[serial_buf_len++] = input;
    if (input != '\n') continue;
    String cmd = String(serial_buf, serial_buf_len);
    serial_buf_len = 0;
    if (cmd.startsWith("pt")) {
      print_string("datetime", rtc.getDateTime());
    } else if (cmd.startsWith("ps")) {
      print_string("ssid", ssid);
    } else if (cmd.startsWith("pz")) {
      print_string("timezone", timezone);
    } else if (cmd.startsWith("pp")) {
      print_string("password", password);
    } else if (cmd.startsWith("ss ")) {
      ssid = String(cmd.substring(3));
      ssid.trim();
      NVS.setString("ssid", ssid);
      print_string("ssid", ssid);
    } else if (cmd.startsWith("sz ")) {
      timezone = String(cmd.substring(3));
      timezone.trim();
      NVS.setString("timezone", timezone);
      print_string("timezone", timezone);
    } else if (cmd.startsWith("sp ")) {
      password = String(cmd.substring(3));
      password.trim();
      NVS.setString("password", password);
      print_string("password", password);
    }
  }
}

void update_display_data() {
  uint8_t h = rtc.getHour(true);
  uint8_t m = rtc.getMinute();
  display_data[0] = seg[h % 10];
  display_data[1] = seg[h / 10];
  display_data[2] = display_dots ? 0xc0 : 0x00;
  display_data[2] |= (0x3f & (seg[m / 10 + 10] >> 2));
  display_data[3] = (((0x03 & seg[m / 10 + 10]) << 6));
  display_data[3] |= (0x3f & (seg[m % 10 + 10] >> 2));
  display_data[4] = (((0x03 & seg[m % 10 + 10]) << 6));
}

inline void cycle_clock() {
  delayMicroseconds(CLOCK_DELAY);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(2 * CLOCK_DELAY);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(CLOCK_DELAY);
}

void write_display_data() {
  cycle_clock();
  digitalWrite(PIN_DAT, HIGH);
  cycle_clock();
  digitalWrite(PIN_DAT, LOW);
  cycle_clock();
  for (int i = 0; i < 34; ++i) {
    digitalWrite(PIN_DAT, display_data[i / 8] & (1 << (7 - (i % 8))));
    cycle_clock();
  }
  digitalWrite(PIN_DAT, LOW);
  for (int i = 0; i < 100; ++i) cycle_clock();
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

  for (int i = 0; i < 100; ++i) cycle_clock();
}

void loop() {
  if ((millis() - last_time_fetch) > time_fetch_interval && has_internet_connection) {
    request_time();
    last_time_fetch = millis();
  }
  if ((millis() - last_time_display) > time_display_interval) {
    last_time_display = millis();
    update_display_data();
    write_display_data();
    display_dots = !display_dots;
  }
  process_serial();
}