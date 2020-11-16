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

/*
  network.h contains network information below:

  const char* ssid     = "SSID";
  const char* password = "PASSWORD";
  const char* host     = "Scratch Host IP";
*/
#include "network.h"

const int Port = 42001;

#include <WiFi.h>
#include <Wire.h>

#if defined(ARDUINO_M5Stick_C)
#include <M5StickC.h>
#elif defined(ARDUINO_M5Stack_Core_ESP32)
// #define M5STACK_MPU6886
#define M5STACK_MPU9250
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include <M5Stack.h>

#if defined(M5STACK_MPU9250)
#include "utility/MPU9250.h"
MPU9250 IMU;
#endif

#define FACES_KEYBOARD_I2C_ADDR 0x08
#elif defined(ARDUINO_M5STACK_Core2)
#include <M5Core2.h>
#elif defined(ARDUINO_M5Stack_ATOM)
#include <M5Atom.h>

uint8_t DisBuff[2 + 5 * 5 * 3];

void setBuff(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata)
{
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++)
  {
    DisBuff[2 + i * 3 + 0] = Rdata;
    DisBuff[2 + i * 3 + 1] = Gdata;
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
}
#endif

#include "utility/MahonyAHRS.h"

WiFiClient client;

void setup() {
  // Init M5
#if defined(ARDUINO_M5Stack_ATOM)
  M5.begin(true, false, true);
#else
  M5.begin();
#endif
  delay(100);

  // Init Serial
  Serial.begin(115200);
  delay(10);

#if !defined(ARDUINO_M5Stack_ATOM)
#if defined(ARDUINO_M5Stick_C)
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(1);
#elif defined(ARDUINO_M5Stack_Core_ESP32)
  M5.Lcd.setTextSize(2);
#endif
  M5.Lcd.println("Welcome to Scratch Remoto Sensor!!");
#endif

#if defined(ARDUINO_M5Stack_ATOM)
  setBuff(0x20, 0x20, 0x20);
  M5.dis.displaybuff(DisBuff);

  // for ATOM Lite
  // M5.dis.drawpix(0, 0xffffff);
#else
  M5.Lcd.println("WiFi connected.");
#endif
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#if !defined(ARDUINO_M5Stack_ATOM)
    M5.Lcd.print(".");
#endif
    Serial.print(".");
  }

  Serial.println("Wifi OK");

  // Wire setup
  Wire.begin();
  pinMode(5, INPUT);
  digitalWrite(5, HIGH);

  // Accel & gyro (& mag for M5Stack)
#if !defined(M5STACK_MPU9250)
  M5.IMU.Init();
#else
  IMU.MPU9250SelfTest(IMU.SelfTest);

  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();

  delay(500);
  IMU.initAK8963(IMU.magCalibration);
#endif

  // LED
#if defined(ARDUINO_M5Stick_C)
  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, LOW);
#endif

  // Button
#if defined(ARDUINO_M5Stick_C)
  pinMode(M5_BUTTON_HOME, INPUT);
  pinMode(M5_BUTTON_RST, INPUT);
#endif

  delay(1000);
}

String getValue(char name, String msg) {
  msg.replace(String(name) + " ", "");
  Serial.println("str:\"" + String(name) + ":" + String(msg) + "\"");
  return msg;
}

void broadcast(String msg) {
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

void sensor_update(String varName, String varValue) {
  char scmd[32] = {0};
  char buf[100] = {0};
  String cmd = "sensor-update \"" + varName + "\" " + varValue + " ";

  cmd.toCharArray(buf, cmd.length() + 1);
  sprintf(scmd + 4, buf);
  scmd[3] = (uint8_t)strlen(scmd + 4);
  delay(10);
  client.setTimeout(100);
  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
    //Serial.println("sensor-update ok");
    return;
  } else {
    //Serial.println("sensor-update err");
    return;
  }
}

void loop() {
  uint8_t buffer[128] = {0};
  int r = 0, g = 0, b = 0;
  String s;
  char* str;

  M5.update();
  delay(10);

  Serial.println("Before client connect");
  while (!client.connect(host, Port)) {
    Serial.println("connection failed");
  }
  Serial.println("create tcp ok");

  while (!client.connected()) {
    Serial.println("Stop connection");
    client.stop();
    Serial.println("Before client.connect");
    //client.setTimeout(100);
    client.connect(host, Port);
    Serial.println("After client.connect");
  }
  Serial.println("Client connected");

  // Read all from server and print them to Serial.
  uint32_t len = 0;
  String msg = "";
  char *c;

  Serial.println("Let us go to read messages.");

  //// Receive msg
  len = 0;
  int av = client.available();
  Serial.println("available:" + String(av));
  //if (av > 0) {
  client.setTimeout(100);
  len = client.readBytes(buffer, sizeof(buffer));
  //}

  Serial.println("Get length:" + String(len));

  while (len > 0) {
#if !defined(ARDUINO_M5Stack_ATOM)
    M5.Lcd.setCursor(0, 0);
#endif
    Serial.print("Received:[");
    // Skip 4 byte message header and get string.
    for (uint32_t i = 4; i < len; i++) {
      Serial.print((char)buffer[i]);
      msg += (char)buffer[i];
    }
    Serial.print("]\r\n");

    while ((!msg.startsWith("broadcast") && !msg.startsWith("sensor-update")) && msg.length() > 0 ) {
      msg = msg.substring(1);
    }

    if (msg.startsWith("broadcast") == true) {
      // message
      msg.replace("broadcast ", "");
      msg.replace("\"", "");
      Serial.println("broadcast:\"" + msg + "\"");
#if !defined(ARDUINO_M5Stack_ATOM)
      M5.Lcd.println("broadcast:\"" + msg + "\"");
#endif
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
            int led =  int(getValue('l', msg).toFloat());
#if defined(ARDUINO_M5Stick_C)
            digitalWrite(M5_LED, led);
#endif
            break;
        }
        //Serial.println("msg:\"" + msg + "\"");

        // Skip var_value
        while (msg.charAt(0) != ' ' && msg.length() > 0) {
          msg = msg.substring(1);
        }
        //Serial.println("msg2:\"" + msg + "\"");
      }

      //// Output
      // RGB background
#if defined(ARDUINO_M5Stack_ATOM)
      // for ATOM Matrix
      // Conver raw rgb(0-255) to led rgb(0-0x20)
      int rl = constrain(int((r / 255.0) * 0x20), 0, 0x20);
      int gl = constrain(int((g / 255.0) * 0x20), 0, 0x20);
      int bl = constrain(int((b / 255.0) * 0x20), 0, 0x20);
      Serial.println("LED RGB:(" + String(rl) + ", " + String(gl) + ", " + String(bl) + ")");

      setBuff(rl, gl, bl);
      M5.dis.displaybuff(DisBuff);

      // for ATOM Lite
      // Conver raw rgb(0-255) to led rgb(0-0xf)
      //rl = constrain(int((r / 255.0) * 0xf), 0, 0xf);
      //gl = constrain(int((g / 255.0) * 0xf), 0, 0xf);
      //bl = constrain(int((b / 255.0) * 0xf), 0, 0xf);
      //M5.dis.drawpix(0, (((gl & 0xf) << 12) | ((rl & 0xf) << 8) | ((bl & 0xf) << 4)));
#else
      M5.Lcd.fillScreen(uint16_t (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3) ));

      M5.Lcd.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
#endif
      Serial.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
      // msg
      //M5.Lcd.setCursor(0, 100);
      //M5.Lcd.setTextSize(5);
#if !defined(ARDUINO_M5Stack_ATOM)
      M5.Lcd.println("s:\"" + s + "\"");
#endif
    } else {
      Serial.println("NOP");
    }

    len = msg.length();
  }

  // broadcast
  broadcast("test");

  // broadcast by button
#if defined(ARDUINO_M5Stick_C)
  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    broadcast("BtnA");
  }
  if (digitalRead(M5_BUTTON_RST) == LOW) {
    broadcast("BtnB");
  }
#elif defined(ARDUINO_M5Stack_Core_ESP32)
  if (M5.BtnA.isPressed()) {
    broadcast("BtnA");
  }
  if (M5.BtnB.isPressed()) {
    broadcast("BtnB");
  }
  if (M5.BtnC.isPressed()) {
    broadcast("BtnC");
  }
#elif defined(ARDUINO_M5Stack_ATOM)
  if (M5.Btn.wasPressed())
  {
    broadcast("Btn");
  }
#endif

#if defined(ARDUINO_M5Stack_Core_ESP32)
  // keyboard input
  if (digitalRead(5) == LOW)
  {
    Wire.requestFrom(FACES_KEYBOARD_I2C_ADDR, 1);
    while (Wire.available())
    {
      char c = Wire.read(); // receive a byte as character
      Serial.print(c);         // print the character
      broadcast("Key_" + String(c));
    }
  }
#endif

  // sensor-update
  sensor_update("v", String(random(0, 255)));


  // define all sensor valiable.
  float ax = 0;
  float ay = 0;
  float az = 0;

  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;

  float mx = 0;
  float my = 0;
  float mz = 0;

  float pitch = 0;
  float roll = 0;
  float yaw = 0;

  float heading = 0;

  float temp = 0;

#if !defined(M5STACK_MPU9250)
  M5.IMU.getAccelData(&ax, &ay, &az);         // get accel
  M5.IMU.getGyroAdc(&gyroX, &gyroY, &gyroZ);  // get gyro
  M5.IMU.getTempData(&temp);                  // get temp
#else
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    // get accel
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();
    ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    // get gyro
    IMU.readGyroData(IMU.gyroCount);
    IMU.getGres();

    gyroX = (float)IMU.gyroCount[0] * IMU.gRes;
    gyroY = (float)IMU.gyroCount[1] * IMU.gRes;
    gyroZ = (float)IMU.gyroCount[2] * IMU.gRes;

    // get magnetic
    IMU.readMagData(IMU.magCount);
    IMU.getMres();

    mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] - 470;
    my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] - 120;
    mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] - 125;

    // get heading
    float offset_mx = 1944.21;
    float offset_my = 1204.83;
    heading = atan2(my - offset_my, mx - offset_mx) * 180.0 / M_PI;

    // get temp
    IMU.tempCount = IMU.readTempData();
    temp = ((float) IMU.tempCount) / 333.87 + 21.0;

    IMU.updateTime();
  }
#endif

  // Calculate pitch, roll, yaw
  MahonyAHRSupdateIMU(gyroX, gyroY, gyroZ, ax, ay, az, &pitch, &roll, &yaw);

  // send sensor-update

  // sensor-update accel
#if defined(ARDUINO_M5Stick_C)
  // Rotation is different from landscape.
  sensor_update("ax", String(-1 * 240 * ay));
  sensor_update("ay", String(+1 * 180 * ax));
#elif defined(ARDUINO_M5Stack_Core_ESP32)
  sensor_update("ax", String(-1 * 240 * ax));
  sensor_update("ay", String(-1 * 180 * ay));
#elif defined(ARDUINO_M5Stack_ATOM)
  sensor_update("ax", String(+1 * 240 * ax));
  sensor_update("ay", String(-1 * 180 * ay));
#endif
  sensor_update("az", String(1000 * az));
#if !defined(ARDUINO_M5Stack_ATOM)
  M5.Lcd.println("accel:(" + String(ax) + ", " + String(ay) + ", " + String(az) + ")");
#endif

  // sensor-update gyro
  sensor_update("gx", String(gyroX));
  sensor_update("gy", String(gyroY));
  sensor_update("gz", String(gyroZ));
#if !defined(ARDUINO_M5Stack_ATOM)
  M5.Lcd.println("gyro:(" + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + ")");
#endif

#if defined(M5STACK_MPU9250)
  // sensor-update magnetic
  sensor_update("mx", String(mx));
  sensor_update("my", String(my));
  sensor_update("mz", String(mz));
#if !defined(ARDUINO_M5Stack_ATOM)
  M5.Lcd.println("mag:(" + String(mx) + ", " + String(my) + ", " + String(mz) + ")");
#endif

  // sensor-update heading
  sensor_update("heading", String(heading));
#if !defined(ARDUINO_M5Stack_ATOM)
  M5.Lcd.println("heading:" + String(heading));
#endif
#endif

  // sensor-update pitch, roll, yaw
  delay(100);
  sensor_update("pitch", String(pitch));
  sensor_update("roll", String(roll));
  sensor_update("yaw", String(yaw));
#if !defined(ARDUINO_M5Stack_ATOM)
  M5.Lcd.println("p,r,y:(" + String(pitch) + ", " + String(roll) + ", " + String(yaw) + ")");
#endif

  // sensor-update temp
#if !defined(ARDUINO_M5Stack_ATOM)
  M5.Lcd.println("temp:" + String(temp));
#endif
  sensor_update("temp", String(temp));

  client.stop();

}
