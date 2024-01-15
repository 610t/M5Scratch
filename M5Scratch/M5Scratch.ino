/*
    Scratch remote sensor protocol

    You need to get ssid , password and host

    http://tiisai.dip.jp/?p=3665
*/

/*
  Copyright 2016,2019-2024 Takeshi MUTOH

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
   This program is demonstration that Scrath Remote Sensor Protocol with M5Stack.
*/

//// Network settings

// If you use fixed network settings below, use #undef WIFI_MODE_PREV
#define WIFI_MODE_PREV  // Use privious settings to setting up Wi-Fi.
// #undef WIFI_MODE_PREV

char* ssid = "SSID";          // Wi-Fi SSID
char* password = "PASSWORD";  // Wi-Fi password or seacret key
char host[128] = { 0 };       // Scratch Host IP
const int Port = 42001;       // Scratch remote sensor port

#include <SD.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <Wire.h>

#define FACES_KEYBOARD_I2C_ADDR 0x08

// For M5Stack Atom's Matrix LED
#include <FastLED.h>
#define NUM_LEDS 25
#define LED_DATA_PIN 27

//// Global variables
m5::board_t myBoard = m5gfx::board_unknown;  // M5Stack board name
CRGB leds[NUM_LEDS];                         // FastLED for M5Stack Atom
WiFiClient client;                           // WiFi connect

//// Draw cat image related.
#include "cat_img.h"  // Scratch cat image.
extern const uint8_t cat[];
int_fast16_t x = 0, y = 0;  // Location
float z;                    // Zoom

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

  // Init FastLED(NeoPixel).
  if (myBoard == m5gfx::board_M5Atom) {
    FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);
  }

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
      Serial.printf("ssid_pos:%d\n", ssid_pos);
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
  } else {
#if defined(WIFI_MODE_PREV)
    WiFi.begin();  // Use privious setting.
#else
    WiFi.begin(ssid, password);  // Use fixed string.
#endif
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

String getValue(char name, String msg) {
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
        msg.trim();
        switch (msg.charAt(0)) {
          case 'x':
            // Cat x axis location
            x = constrain(int(getValue('x', msg).toFloat()), -240, 240);
            break;
          case 'y':
            // Cat y axis location
            y = constrain(int(getValue('y', msg).toFloat()), -180, 180);
            break;
          case 'z':
            // Cat zoom value
            z = constrain(getValue('z', msg).toFloat(), 1, 10);
            break;
          case 'r':
            r = constrain(int(getValue('r', msg).toFloat()), 0, 255);
            break;
          case 'g':
            g = constrain(int(getValue('g', msg).toFloat()), 0, 255);
            break;
          case 'b':
            b = constrain(int(getValue('b', msg).toFloat()), 0, 255);
            break;
          case 's':
            s = getValue('s', msg);
            break;
          case 'l':
            int led = int(getValue('l', msg).toFloat());
            M5.Power.setLed(constrain(led, 0, 255));
            break;
        }
        //log_i("msg:\"%s\"\n",msg);

        // Skip var_value
        while (msg.charAt(0) != ' ' && msg.length() > 0) {
          msg = msg.substring(1);
        }
        //log_i("msg2:\"%s\"\n",msg);
      }

      send_M5Stack_data();

      //// Output
      // RGB background
      // Fill background (r,g,b) for ATOM Matrix
      if (myBoard == m5gfx::board_M5Atom) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(r, g, b);
        }
        FastLED.show();
      }

      // Fill background (r,g,b) for other boards.
      int rgb = uint16_t(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
      M5.Lcd.fillScreen(rgb);

      // Draw top right circle
      // int circle_r = 40;
      // M5.Lcd.fillCircle(M5.Lcd.display.width() - circle_r, circle_r, circle_r, rgb);

      M5.Lcd.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
      log_i("RGB:(%d,%d,%d)\n", r, g, b);

      // Draw cat
      M5.Lcd.drawPng(cat, ~0u,                                 // Data
                     x, y,                                     // Position
                     M5.Lcd.width() * 2, M5.Lcd.height() * 2,  // Size
                     0, 0,                                     // Offset
                     z, 0,                                     // Magnify
                     datum_t::middle_center);

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
