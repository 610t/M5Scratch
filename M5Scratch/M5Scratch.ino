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

/*
  network.h contains network information below:

  const char* ssid     = "SSID";
  const char* password = "PASSWORD";
  const char* host     = "Scratch Host IP";
*/
#include "network.h"

const int Port = 42001;

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
String r = "";                               // String received
WiFiClient client;                           // WiFi connect

void setup() {
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

  // Init Serial
  Serial.begin(115200);
  delay(10);

  M5.Lcd.println("Welcome to Scratch Remoto Sensor!!");

  M5.Lcd.println("WiFi connected.");
  log_i("Connecting to %s\n", ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
    log_i(".");
  }

  log_i("Wifi OK\n");

  // Wire setup
  Wire.begin();
  pinMode(5, INPUT);
  digitalWrite(5, HIGH);

  log_i("Scratch Host IP is {%s}\n", host);
  delay(1000);
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
  delay(10);
  client.setTimeout(100);
  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
    //log_i("sensor-update ok\n");
    return;
  } else {
    //log_i("sensor-update err\n");
    return;
  }
}

void loop() {
  uint8_t buffer[128] = { 0 };
  int r = 0, g = 0, b = 0;
  String s;
  char* str;

  M5.update();

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

  // Read all from server and print them to Serial.
  uint32_t len = 0;
  String msg = "";
  char* c;

  log_i("Let us go to read messages.\n");

  //// Receive msg
  len = 0;
  int av = client.available();
  log_i("available:%d\n", av);
  //if (av > 0) {
  client.setTimeout(100);
  len = client.readBytes(buffer, sizeof(buffer));
  //}

  log_i("Get length:%d\n", len);

  while (len > 0) {
    M5.Lcd.setCursor(0, 0);
    log_i("Received:[");
    // Skip 4 byte message header and get string.
    for (uint32_t i = 4; i < len; i++) {
      log_i("%c", (char)buffer[i]);
      msg += (char)buffer[i];
    }
    log_i("]\n");

    while ((!msg.startsWith("broadcast") && !msg.startsWith("sensor-update")) && msg.length() > 0) {
      msg = msg.substring(1);
    }

    if (msg.startsWith("broadcast") == true) {
      // message
      msg.replace("broadcast ", "");
      msg.replace("\"", "");
      log_i("broadcast:\"%s\"\n", msg);
      M5.Lcd.println("broadcast:\"" + msg + "\"");
    } else if (msg.startsWith("sensor-update")) {
      // value
      msg.replace("sensor-update ", "");
      msg.replace("\"", "");
      msg.trim();
      //M5.Lcd.println("sensor-update\"" + msg + "\"");

      while (msg.length() > 0) {
        msg.trim();
        switch (msg.charAt(0)) {
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
      M5.Lcd.fillScreen(uint16_t(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)));

      M5.Lcd.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
      log_i("RGB:(%d,%d,%d)\n", r, g, b);
      // msg
      //M5.Lcd.setCursor(0, 100);
      //M5.Lcd.setTextSize(5);
      M5.Lcd.println("s:\"" + s + "\"");
    } else {
      log_i("NOP\n");
    }

    len = msg.length();
  }

  // broadcast
  broadcast("test");

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

  if (myBoard == m5gfx::board_M5Stack) {
    // keyboard input
    if (digitalRead(5) == LOW) {
      Wire.requestFrom(FACES_KEYBOARD_I2C_ADDR, 1);
      while (Wire.available()) {
        char c = Wire.read();  // receive a byte as character
        log_i("%c", c);        // print the character
        broadcast("Key_" + String(c));
      }
    }
  }

  // sensor-update
  sensor_update("v", String(random(0, 255)));


  // define all sensor valiable.
  float ax = 0;
  float ay = 0;
  float az = 0;

  float gx = 0;
  float gy = 0;
  float gz = 0;

  float pitch = 0;
  float roll = 0;
  float yaw = 0;

  float heading = 0;

  float temp = 0;

  M5.Imu.getAccel(&ax, &ay, &az);  // get accel
  M5.Imu.getGyro(&gx, &gy, &gz);   // get gyro
  M5.Imu.getTemp(&temp);           // get temperature

  //// send sensor-update
  // sensor-update accel
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
  delay(100);
  sensor_update("pitch", String(pitch));
  sensor_update("roll", String(roll));
  sensor_update("yaw", String(yaw));
  M5.Lcd.println("p,r,y:(" + String(pitch) + ", " + String(roll) + ", " + String(yaw) + ")");

  // sensor-update temp
  M5.Lcd.println("temp:" + String(temp));
  sensor_update("temp", String(temp));

  client.stop();
}
