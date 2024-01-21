/*
  Scratch remote sensor protocol demonstration

  This program is demonstration that Scrath Remote Sensor Protocol for M5Stack family.

  This software is provided at BSD 2-Clause "Simplified" License.
  Please see more detail: https://github.com/610t/M5Scratch/blob/master/LICENSE .
  Original work is at http://tiisai.dip.jp/?p=3665 .
*/

//// Network settings

// If you use fixed network settings below, use #undef WIFI_MODE_PREV and set ssid, password, and host.
//  This setting is useful for no SD machine such like Atom series, M5Stick series and M5Dial.
#define WIFI_MODE_PREV  // Use privious settings to setting up Wi-Fi.
// #undef WIFI_MODE_PREV

char* ssid = "SSID";          // Wi-Fi SSID
char* password = "PASSWORD";  // Wi-Fi password or seacret key
#if !defined(WIFI_MODE_PREV)
char* host = "Scratch host IP";  // Scratch Host IP: If you want to use fixed IP, please use this.
#else
char host[128] = { 0 };          // Scratch Host IP: for SD card setting.
#endif
const int Port = 42001;  // Scratch remote sensor port

#include <SD.h>
#include <M5Unified.h>
#include <M5Dial.h>
#include <nvs.h>
#include <WiFi.h>
#include <Wire.h>

#define FACES_KEYBOARD_I2C_ADDR 0x08

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
// For M5Stack Atom's Matrix LED
#include <FastLED.h>
#define NUM_LEDS 25
#define LED_DATA_PIN 27
CRGB leds[NUM_LEDS];  // FastLED for M5Stack Atom
#endif

//// Global variables
m5::board_t myBoard = m5gfx::board_unknown;  // M5Stack board name
WiFiClient client;                           // WiFi connect

//// Draw images.
// Draw cat image.
#include "cat_img.h"  // Scratch cat image.
extern const uint8_t cat[];
bool cat_flag = false;      // Show cat?
int_fast16_t x = 0, y = 0;  // Location
float z;                    // Zoom

// Draw stackchan image.
#include "stackchan_img.h"  // Scratch cat image.
extern const uint8_t stackchan[];
bool stackchan_flag = false;

void setup_M5Stack() {
  // Init M5
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.init();

  // Init speaker.
  auto spk_cfg = M5.Speaker.config();
  M5.Speaker.config(spk_cfg);
  M5.Speaker.begin();
  myBoard = M5.getBoard();

  // Setup M5Dial
  if (myBoard == m5gfx::board_M5Dial) {
    M5Dial.begin(cfg, true, false);
  }

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  // Init FastLED(NeoPixel).
  if (myBoard == m5gfx::board_M5Atom) {
    FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);
  }
#endif

  // Wire setup for M5Stack face keyboard.
  Wire.begin();
  pinMode(5, INPUT);
  digitalWrite(5, HIGH);
}

void setup_WiFi() {
  M5.Lcd.println("WiFi connected.");
  log_i("Connecting to %s\n", ssid);

  //// Wi-Fi & Host IP settings.
  if (SD.begin(GPIO_NUM_4, SPI, 25000000)) {
    // Setting up network information from SD card file "/m5scratch.txt".
    auto fs = SD.open("/m5scratch.txt", FILE_READ);
    if (fs) {
      size_t sz = fs.size();
      char buf[sz + 1];
      fs.read((uint8_t*)buf, sz);
      buf[sz] = 0;
      fs.close();

      // Replace '\n' with 0 to end in NULL.
      for (int x = 0; x < sz; x++) {
        if (buf[x] == 0x0a || buf[x] == 0x0d) {
          buf[x] = 0;
        }
      }

      int ssid_pos = 0;
      int hostip_pos = 0;
      for (ssid_pos = 0; buf[ssid_pos] != 0; ssid_pos++) {}
      ssid_pos++;
      log_i("ssid_pos:%d\n", ssid_pos);
      for (hostip_pos = ssid_pos; buf[hostip_pos] != 0; hostip_pos++) {}
      hostip_pos++;
      WiFi.begin(buf, &buf[ssid_pos]);

      // Seting up Scratch host IP from SD card.
      int i = 0;
      for (int c = hostip_pos; buf[c] != 0; c++) {
        host[i] = buf[c];
        i++;
      }
      host[i] = 0;  // NULL terminate.
    }

    // Save Scratch host IP to NVS.
    uint32_t nvs_handle;
    if (ESP_OK == nvs_open("ScratchHost", NVS_READWRITE, &nvs_handle)) {
      nvs_set_str(nvs_handle, "ScratchHost", host);
    }
  } else {
#if defined(WIFI_MODE_PREV)
    WiFi.begin();  // Use privious setting.
#else
    WiFi.begin(ssid, password);  // Use fixed string.
#endif
  }

  {
    // Read Scratch host IP from NVS.
    uint32_t nvs_handle;

    if (ESP_OK == nvs_open("ScratchHost", NVS_READONLY, &nvs_handle)) {
      size_t length;
      if (ESP_OK == nvs_get_str(nvs_handle, "ScratchHost", nullptr, &length) && length) {
        char scratchhost_ip[length + 1];
        if (ESP_OK == nvs_get_str(nvs_handle, "ScratchHost", scratchhost_ip, &length)) {
          int i;
          for (i = 0; i < length; i++) {
            host[i] = scratchhost_ip[i];
          }
          host[i] = 0;  // NULL terminate
          log_i("Host IP:%s\n", host);
        }
      }
      nvs_close(nvs_handle);
    }
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
    log_i(".");
  }

  log_i("Wifi OK\n");
}

void setup() {
  // Init Serial
  Serial.begin(115200);
  setup_M5Stack();
  M5.Lcd.println("Welcome to Scratch Remoto Sensor!!");
  setup_WiFi();
  log_i("Scratch Host IP is {%s}\n", host);
}

String getValue(char* name, String msg) {
  msg.replace(String(name) + " ", "");
  log_i("str:\"%s:%s\"\n", name, msg);
  return msg;
}

void broadcast(String msg) {
  char scmd[32] = { 0 };
  char buf[100] = { 0 };
  String cmd = "broadcast \"" + msg + "\"";

  cmd.toCharArray(buf, cmd.length() + 1);
  strcpy(scmd + 4, buf);
  //scmd[3] = (uint8_t)strlen(scmd + 4);
  scmd[3] = cmd.length();
  log_i(">pre broadcast:%s\n", scmd + 4);
  client.setTimeout(100);
  //  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
  if (client.write((const uint8_t*)scmd, 4 + cmd.length())) {
    log_i(">>broadcast:%s ok\n", msg);
  } else {
    log_i(">>broadcast:%s err\n", msg);
  }
}

void sensor_update(String varName, String varValue) {
  char scmd[32] = { 0 };
  char buf[100] = { 0 };
  String cmd = "sensor-update \"" + varName + "\" " + varValue + " ";

  cmd.toCharArray(buf, cmd.length() + 1);
  sprintf(scmd + 4, buf);
  scmd[3] = (uint8_t)strlen(scmd + 4);
  client.setTimeout(100);
  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
    //log_i("sensor-update ok\n");
    return;
  } else {
    //log_i("sensor-update err\n");
    return;
  }
}

void client_connect() {
  log_i("Before client connect\n");
  while (!client.connect(host, Port)) {
    log_i("Scratch Host IP is {%s}\n", host);
    log_i("connection failed\n");
  }
  log_i("create tcp ok\n");

  while (!client.connected()) {
    log_i("Stop connection\n");
    client.stop();
    log_i("Before client.connect\n");
    //client.setTimeout(100);
    client.connect(host, Port);
    log_i("After client.connect\n");
  }
  log_i("Client connected\n");
}

void send_broadcast() {
  broadcast("test");  // 'test' event

  // broadcast by button
  if (M5.BtnA.isPressed()) {
    broadcast("BtnA");
  }
  if (M5.BtnB.isPressed()) {
    broadcast("BtnB");
  }
  if (M5.BtnC.isPressed()) {
    broadcast("BtnC");
  }

  // keyboard input
  if (myBoard == m5gfx::board_M5Stack) {

    if (digitalRead(5) == LOW) {
      Wire.requestFrom(FACES_KEYBOARD_I2C_ADDR, 1);
      while (Wire.available()) {
        char c = Wire.read();  // receive a byte as character
        log_i("%c", c);        // print the character
        broadcast("Key_" + String(c));
      }
    }
  }
}

void send_sensor_update() {
  // define all sensor valiable.
  float ax = 0,
        ay = 0, az = 0;                // Accel
  float gx = 0, gy = 0, gz = 0;        // Gyro
  float pitch = 0, roll = 0, yaw = 0;  // Posure
  float temp = 0;                      // Temperature

  M5.Imu.getAccel(&ax, &ay, &az);  // get accel
  M5.Imu.getGyro(&gx, &gy, &gz);   // get gyro
  M5.Imu.getTemp(&temp);           // get temperature

  // Encoder for M5Dial
  if (myBoard == m5gfx::board_M5Dial) {
    long pos = M5Dial.Encoder.read();
    sensor_update("e", String(pos));
    auto t = M5Dial.Touch.getDetail();
    sensor_update("tx", String(t.x));
    sensor_update("ty", String(t.y));
  } else {
    auto t = M5.Touch.getDetail();
    sensor_update("tx", String(t.x));
    sensor_update("ty", String(t.y));
  }

  sensor_update("v", String(random(0, 255)));  // random number 'v'
  // sensor-update accel: normarize to fit for Scratch display.
  sensor_update("ax", String(-1 * 240 * ay));
  sensor_update("ay", String(+1 * 180 * ax));
  sensor_update("az", String(1000 * az));
  M5.Lcd.println("accel:(" + String(ax) + ", " + String(ay) + ", " + String(az) + ")");

  // sensor-update gyro
  sensor_update("gx", String(gx));
  sensor_update("gy", String(gy));
  sensor_update("gz", String(gz));
  M5.Lcd.println("gyro:(" + String(gx) + ", " + String(gy) + ", " + String(gz) + ")");

  // sensor-update pitch, roll, yaw
  sensor_update("pitch", String(pitch));
  sensor_update("roll", String(roll));
  sensor_update("yaw", String(yaw));
  M5.Lcd.println("p,r,y:(" + String(pitch) + ", " + String(roll) + ", " + String(yaw) + ")");

  // sensor-update temp
  M5.Lcd.println("temp:" + String(temp));
  sensor_update("temp", String(temp));
}

void send_M5Stack_data() {
  send_broadcast();
  send_sensor_update();
}

int receive_msg(uint8_t* buffer) {
  uint8_t header[4] = { 0 };
  int len = 0;

  int av = client.available();
  log_i("available:%d\n", av);
  //if (av > 0) {
  client.setTimeout(100);
  client.readBytes(header, 4);  // Read length of command.
  uint32_t cmd_size = header[3] | (header[2] << 8) | (header[1] << 16) | (header[0] << 24);
  len = client.readBytes(buffer, cmd_size);
  //}
  return (len);
}

String create_msg(uint8_t* buffer, int len) {
  String msg = "";

  for (uint32_t i = 0; i < len; i++) {
    if (buffer[i] != '"') {  // Skip '"'
      msg += (char)buffer[i];
    }
  }

  return (msg);
}

void loop() {
  uint8_t buffer[512] = { 0 };
  int r = 0, g = 0, b = 0;
  String s;

  M5.update();
  if (myBoard == m5gfx::board_M5Dial) {
    M5Dial.update();
  }

  client_connect();

  // Read all from server and print them to Serial.
  String msg = "";

  log_i("Let us go to read messages.\n");

  int rcv_len = receive_msg(buffer);
  msg = create_msg(buffer, rcv_len);
  int len = msg.length();

  while (len > 0) {
    M5.Lcd.setCursor(0, 0);

    if (msg.startsWith("broadcast") == true) {
      // message
      log_i("broadcast:\"%s\"\n", msg);
      M5.Lcd.println("broadcast:\"" + msg + "\"");
    } else if (msg.startsWith("sensor-update")) {
      // value
      msg.replace("sensor-update ", "");
      //M5.Lcd.println("sensor-update\"" + msg + "\"");

      while (msg.length() > 0) {
        char cmd_str[512] = { 0 };

        //// Command handler
        // Get command
        msg.trim();
        int i = 0;
        for (i = 0; i < msg.length() && msg.charAt(i) != ' '; i++) {
          cmd_str[i] = msg.charAt(i);
        }
        cmd_str[i] = 0;
        log_i("CMD STR:%s\n", cmd_str);

        // Do command.
        if (!strcmp(cmd_str, "x")) {
          x = constrain(int(getValue("x", msg).toFloat()), -240, 240);
        } else if (!strcmp(cmd_str, "y")) {
          y = constrain(int(getValue("y", msg).toFloat()), -180, 180);
        } else if (!strcmp(cmd_str, "z")) {
          z = constrain(getValue("z", msg).toFloat(), 1, 10);
        } else if (!strcmp(cmd_str, "r")) {
          r = constrain(int(getValue("r", msg).toFloat()), 0, 255);
        } else if (!strcmp(cmd_str, "g")) {
          g = constrain(int(getValue("g", msg).toFloat()), 0, 255);
        } else if (!strcmp(cmd_str, "b")) {
          b = constrain(int(getValue("b", msg).toFloat()), 0, 255);
        } else if (!strcmp(cmd_str, "s")) {
          s = getValue("s", msg);
        } else if (!strcmp(cmd_str, "l")) {
          int led = int(getValue("l", msg).toFloat());
          M5.Power.setLed(constrain(led, 0, 255));
        } else if (!strcmp(cmd_str, "cat")) {
          cat_flag = (!(getValue("cat", msg).toInt() == 0));
        } else if (!strcmp(cmd_str, "stackchan")) {
          stackchan_flag = (!(getValue("stackchan", msg).toInt() == 0));
        }
        log_i("msg:\"%s\"\n", msg);

        // Skip var_value
        while (msg.charAt(0) != ' ' && msg.length() > 0) {
          msg = msg.substring(1);
        }
      }

      send_M5Stack_data();

      //// Output
      // RGB background

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
      // Fill background (r,g,b) for ATOM Matrix
      if (myBoard == m5gfx::board_M5Atom) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(r, g, b);
        }
        FastLED.show();
      }
#endif

      // Fill background (r,g,b) for other boards.
      int rgb = uint16_t(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
      M5.Lcd.fillScreen(rgb);

      // Draw top right circle
      // int circle_r = 40;
      // M5.Lcd.fillCircle(M5.Lcd.display.width() - circle_r, circle_r, circle_r, rgb);

      M5.Lcd.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
      log_i("RGB:(%d,%d,%d)\n", r, g, b);

      // Draw cat
      if (cat_flag) {
        M5.Lcd.drawPng(cat, ~0u,                         // Data
                       x, y,                             // Position
                       M5.Lcd.width(), M5.Lcd.height(),  // Size
                       0, 0,                             // Offset
                       z, 0,                             // Magnify
                       datum_t::top_left);
      }

      // Draw Stackchan
      if (stackchan_flag) {
        M5.Lcd.drawPng(stackchan, ~0u,                   // Data
                       x, y,                             // Position
                       M5.Lcd.width(), M5.Lcd.height(),  // Size
                       0, 0,                             // Offset
                       z, 0,                             // Magnify
                       datum_t::top_left);
      }

      // msg
      //M5.Lcd.setCursor(0, 100);
      //M5.Lcd.setTextSize(5);
      M5.Lcd.println("s:\"" + s + "\"");
    } else {
      log_i("NOP\n");
    }

    len = msg.length();
  }

  client.stop();
}
