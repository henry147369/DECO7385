#ifndef MOVESENSOR_H
#define MOVESENSOR_H

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

#define INTERRUPT_PIN 15

extern MPU6050 mpu;
extern bool dmpReady;
extern uint16_t packetSize;
extern uint8_t fifoBuffer[64];
extern Quaternion q;
extern VectorFloat gravity;
extern float ypr[3];

extern unsigned long lastSuccess;
extern unsigned long lastReconnectAttempt;
extern int failedReadCount;
const int MAX_FAILED_READS = 5;
const int RECONNECT_DELAY = 500;

bool leftRightTriggered = false;
bool forwardBackTriggered = false;
unsigned long leftRightTriggerTime = 0;
unsigned long forwardBackTriggerTime = 0;
const unsigned long TRIGGER_COOLDOWN = 2000;

bool mpuPermanentlyFailed = false;


bool initMPU() {
  mpu.reset();
  delay(50);
  mpu.initialize();
  if (!mpu.testConnection()) return false;
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) return false;
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  return true;
}


void setupMPU() {
  pinMode(INTERRUPT_PIN, INPUT);

  if (!initMPU()) {
    Serial.println("MPU6050初始化失败！尝试重新连接...");
    delay(1000);
    if (!initMPU()) {
      Serial.println("第二次初始化失败。将进入降级模式并等待后续自动恢复。");
      dmpReady = false;
      lastReconnectAttempt = millis(); // 允许loop中定时恢复
      return;
    }
  }
  Serial.println("MPU6050初始化成功！");
  lastSuccess = millis();
}


bool safeReadMPU(unsigned long timeout = 30) {
  unsigned long start = millis();
  while (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    if (millis() - start > timeout) {
      failedReadCount++;
      return false;
    }
    delay(5); // 避免 busy-loop
  }
  failedReadCount = 0;
  return true;
}


void updateRemoteMotionSensor() {
  while (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    msg.trim();  // 清除空白字符

    Serial.println("Received raw: " + msg);

    if (msg.length() == 0 || !msg.startsWith("B")) continue;

    int colonIndex = msg.indexOf(':');
    if (colonIndex == -1) continue;

    int sensorID = msg.substring(1, colonIndex).toInt();
    String direction = msg.substring(colonIndex + 1);
    unsigned long now = millis();

    if (direction == "LeftRight" && (now - leftRightTriggerTime > TRIGGER_COOLDOWN)) {
      Serial.print("Remote Sensor B"); Serial.print(sensorID); Serial.println(" → LEFT");
      leftRightTriggered = true;
      leftRightTriggerTime = now;
    } else if (direction == "ForwardBack" && (now - forwardBackTriggerTime > TRIGGER_COOLDOWN)) {
      Serial.print("Remote Sensor B"); Serial.print(sensorID); Serial.println(" → FORWARD");
      forwardBackTriggered = true;
      forwardBackTriggerTime = now;
    }
  }
}


void updateMotionSensor() {
  updateRemoteMotionSensor(); // Receive signal from sub-board
  
  if (!safeReadMPU()) {
      static bool warned = false;
      if (!warned) {
        Serial.println("[警告] MPU读取失败，将跳过该帧");
        warned = true; // 仅打印一次警告
      }
      return; // ⛔️ 跳过当前帧，继续 loop
  }

  // 恢复状态（如果曾失败）
  failedReadCount = 0;
  lastSuccess = millis();
  mpuPermanentlyFailed = false;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float pitch = ypr[1] * 180 / M_PI;
  float roll = ypr[2] * 180 / M_PI;
  unsigned long now = millis();

  if ((roll > 20 || roll < -20) && (now - leftRightTriggerTime > TRIGGER_COOLDOWN)) {
    leftRightTriggered = true;
    leftRightTriggerTime = now;
  }
  if ((pitch > 20 || pitch < -20) && (now - forwardBackTriggerTime > TRIGGER_COOLDOWN)) {
    forwardBackTriggered = true;
    forwardBackTriggerTime = now;
  }
  if (abs(roll) < 5) leftRightTriggered = false;
  if (abs(pitch) < 5) forwardBackTriggered = false;
}


void reconnectIfNeeded() {
  if (!dmpReady && millis() - lastReconnectAttempt > RECONNECT_DELAY) {
    lastReconnectAttempt = millis();
    if (initMPU()) {
      failedReadCount = 0;
      lastSuccess = millis();
    }
  }
}


#endif