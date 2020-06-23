/*
    Scratch remote sensor protocol

    You need to get ssid , password and host

    http://tiisai.dip.jp/?p=3665
*/

/*
  Copyright 2016,2019-2020 Takeshi MUTOH

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

#include <WiFi.h>
#include <M5Stack.h>
#include <Wire.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define FACES_KEYBOARD_I2C_ADDR 0x08
MPU9250 IMU;

/*
  network.h contains network information below:

  const char* ssid     = "SSID";
  const char* password = "PASSWORD";
  const char* host     = "Scratch Host IP";
*/
#include "network.h"

const int Port = 42001;

void WiFiSetup() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    Serial.println("Wifi restart.");
  } else {
    Serial.println("Wifi start.");
  }

  WiFi.disconnect();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.println("");
}

void setup() {
  // Init M5
  M5.begin();
  delay(100);

  // Init Serial
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network
  M5.Lcd.println("Welcome to Scratch Remoto Sensor!!");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFiSetup();

  M5.Lcd.println("WiFi connected.");

  Serial.println("Wifi OK");

  // Wire setup
  Wire.begin();
  pinMode(5, INPUT);
  digitalWrite(5, HIGH);

  // Accel
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  IMU.MPU9250SelfTest(IMU.SelfTest);
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

  IMU.initMPU9250();
  byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  IMU.initAK8963(IMU.magCalibration);

  delay(5000);
}

String getValue(char name, String msg) {
  msg.replace(String(name) + " ", "");
  Serial.println("{" + String(name) + ":" + String(msg) + "}");
  return msg;
}

void broadcast(WiFiClient client, String msg) {
  char scmd[32] = {0};
  char buf[100] = {0};
  String cmd = "broadcast \"" + msg + "\"";

  cmd.toCharArray(buf, cmd.length() + 1);
  strcpy(scmd + 4, buf);
  //scmd[3] = (uint8_t)strlen(scmd + 4);
  scmd[3] = cmd.length();
  Serial.println(">pre broadcast:" + String(scmd + 4));
  client.setTimeout(100);
  //  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
  if (client.write((const uint8_t*)scmd, 4 + cmd.length())) {
    Serial.println(">>broadcast:" + msg + " ok");
  } else {
    Serial.println(">>broadcast:" + msg + " err");
  }
}

void sensor_update(WiFiClient client, String varName, String varValue) {
  char scmd[32] = {0};
  char buf[100] = {0};
  String cmd = "sensor-update \"" + varName + "\" " + varValue + " ";

  cmd.toCharArray(buf, cmd.length() + 1);
  sprintf(scmd + 4, buf);
  scmd[3] = (uint8_t)strlen(scmd + 4);
  client.setTimeout(100);
  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
    Serial.println("sensor-update ok");
  } else {
    Serial.println("sensor-update err");
  }
}

WiFiClient client;

void loop() {
  uint8_t buffer[128] = {0};
  int r = 0, g = 0, b = 0;
  String s;
  char* str;

  M5.update();
  delay(10);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi status Ok:");
  } else {
    Serial.println("WiFi not connected");
  }

  // Use WiFiClient class to create TCP connections
  if (!client.connect(host, Port)) {
    Serial.println("connection failed");
    return;
  }
  Serial.print("create tcp ok\r\n");

  while (!client.connected()) {
    WiFiSetup();
    client.connect(host, Port);
  }

  // Read all from server and print them to Serial.
  uint32_t len = 0;
  String msg = "";
  char *c;

  //// Receive msg
  client.setTimeout(100);
  len = client.readBytes(buffer, sizeof(buffer));

  Serial.println("Get length:" + String(len));

  while (len > 0) {
    M5.Lcd.setCursor(0, 0);
    Serial.print("Received:[");
    for (uint32_t i = 0; i < len; i++) {
      Serial.print((char)buffer[i]);
      if (i >= 4) { // Skip 4 byte message header
        msg += (char)buffer[i];
      }
    }
    Serial.print("]\r\n");

    while ((!msg.startsWith("broadcast") && !msg.startsWith("sensor-update")) && msg.length() > 0 ) {
      msg.substring(1);
    }

    if (msg.startsWith("broadcast") == true) {
      // message
      msg.replace("broadcast ", "");
      msg.replace("\"", "");
      Serial.println("{broadcast:" + msg + "}");
      M5.Lcd.setTextSize(3);
      M5.Lcd.println("{broadcast:" + msg + "}");
    } else if (msg.startsWith("sensor-update")) {
      // value
      msg.replace("sensor-update ", "");
      msg.replace("\"", "");
      msg.trim();
      //M5.Lcd.println("{sensor-update " + msg + "}");

      while (msg.length() > 0) {
        msg.trim();
        switch (msg.charAt(0)) {
          case 'r':
            r = int(getValue('r', msg).toFloat());
            break;
          case 'g':
            g = int(getValue('g', msg).toFloat());
            break;
          case 'b':
            b = int(getValue('b', msg).toFloat());
            break;
          case 's':
            s = getValue('s', msg);
            break;
        }
        Serial.println("{{msg:" + msg + "}}");

        // Skip var_value
        while (msg.charAt(0) != ' ' && msg.length() > 0) {
          msg = msg.substring(1);
        }
        Serial.println("{{msg2:" + msg + "}}");
      }

      //// Output
      // RGB background
      M5.Lcd.fillScreen(uint16_t (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3) ));

      M5.Lcd.setTextSize(2);
      M5.Lcd.println("{RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")}");
      Serial.println("{RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")}");
      // msg
      //M5.Lcd.setCursor(0, 100);
      //M5.Lcd.setTextSize(5);
      M5.Lcd.println("{s:" + s + "}");
    } else {
      Serial.println("NOP");
    }

    len = msg.length();
  }

  // broadcast
  broadcast(client, "test");
  if (M5.BtnA.isPressed()) {
    broadcast(client, "BtnA1");
  }
  if (M5.BtnB.isPressed()) {
    broadcast(client, "BtnB1");
  }
  if (M5.BtnC.isPressed()) {
    broadcast(client, "BtnC1");
  }

  // keyboard input
  if (digitalRead(5) == LOW)
  {
    Wire.requestFrom(FACES_KEYBOARD_I2C_ADDR, 1);
    while (Wire.available())
    {
      char c = Wire.read(); // receive a byte as character
      Serial.print(c);         // print the character
      broadcast(client, "Key_" + String(c));
    }
  }

  // sensor-update
  sensor_update(client, "v1", String(random(0, 255)));

  // sensor-update by accel
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    sensor_update(client, "ax1", String(-1 * 240 * IMU.ax));
    sensor_update(client, "ay1", String(-1 * 180 * IMU.ay));
    sensor_update(client, "az1", String(1000 * IMU.az));
    M5.Lcd.setTextSize(2);
    M5.Lcd.println("{ax,ay,az:(" + String(IMU.ax) + ", " + String(IMU.ay) + ", " + String(IMU.az) + ")}");
  }
}
