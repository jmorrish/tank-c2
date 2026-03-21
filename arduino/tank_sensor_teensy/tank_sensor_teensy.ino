// ============================================================
// Tank Sensor Teensy  —  IP 192.168.0.178 : 23
//
// Handles: IMU (BNO08x on Serial5), GPS (raw NMEA on Serial6),
//          TOF distance sensor (VL53L0X on I2C)
//
// Commands received from Jetson (newline-terminated):
//   IMU_ON          — start streaming YPR at current rate
//   IMU_OFF         — stop streaming YPR
//   IMU_RATE <hz>   — set YPR output rate (1–200 Hz)
//
// Data sent to Jetson (newline-terminated):
//   YPR <yaw> <pitch> <roll>    — degrees, from BNO08x rotation vector
//   TOF <dist_m>                — metres, from VL53L0X (or -1 if invalid)
//   $GP... raw NMEA sentences   — forwarded verbatim from GPS module
//
// Libraries required (install via Arduino Library Manager):
//   Adafruit BNO08x         (Adafruit)
//   Adafruit VL53L0X        (Adafruit)
//   NativeEthernet          (Teensy 4.1)
// ============================================================

#include <NativeEthernet.h>
#include "Adafruit_BNO08x.h"
#include <Adafruit_VL53L0X.h>
#include <math.h>
#include <Wire.h>

// ------- Ethernet -------
byte      mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF };  // different MAC from control Teensy
IPAddress ip(192, 168, 0, 178);
EthernetServer server(23);
EthernetClient client;

// ------- IMU (BNO08x on Serial5) -------
Adafruit_BNO08x bno08x;
const uint32_t BNO_UART_BAUD = 115200;
bool     imuStreaming = false;
uint32_t yprPeriodMs = 20;   // default 50 Hz
uint32_t lastYprSend = 0;

// ------- GPS (raw NMEA passthrough on Serial6) -------
HardwareSerial& gpsSerial = Serial6;
char nmeaBuffer[256];
int  nmeaLen = 0;

// ------- TOF (VL53L0X on I2C) -------
Adafruit_VL53L0X tof;
bool     tofReady = false;
uint32_t lastTofSend = 0;
const uint32_t TOF_PERIOD_MS = 100;  // 10 Hz distance updates

// ------- Command line buffer -------
char   cmdBuf[128];
size_t cmdLen = 0;

// Quaternion → YPR (degrees)
static void quatToYPR(float qx, float qy, float qz, float qw,
                      float& yaw, float& pitch, float& roll) {
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float sinp = 2.0f * (qw * qy - qz * qx);
  pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * 180.0f / PI;

  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

// Send a line to the connected client (if any)
static void clientPrintln(const char* s) {
  if (client && client.connected() && client.availableForWrite() >= (int)strlen(s) + 2)
    client.println(s);
}

// Process one complete command line
void processCommand(const char* cmd) {
  Serial.print("CMD: "); Serial.println(cmd);

  if (strcmp(cmd, "IMU_ON") == 0) {
    imuStreaming = true;
    clientPrintln("C IMU Streaming ON");
    return;
  }
  if (strcmp(cmd, "IMU_OFF") == 0) {
    imuStreaming = false;
    clientPrintln("C IMU Streaming OFF");
    return;
  }
  // IMU_RATE <hz>
  if (strncmp(cmd, "IMU_RATE", 8) == 0 && (cmd[8] == ' ' || cmd[8] == '\0')) {
    float hz = atof(cmd + 9);
    if (hz < 1.0f)   hz = 1.0f;
    if (hz > 200.0f) hz = 200.0f;
    yprPeriodMs = (uint32_t)(1000.0f / hz);
    char resp[40];
    snprintf(resp, sizeof(resp), "C IMU Rate = %.1f Hz", hz);
    clientPrintln(resp);
    return;
  }
  clientPrintln("ERR Unrecognized Command");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ----- Ethernet -----
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("Sensor Teensy listening on ");
  Serial.print(ip); Serial.println(":23");

  // ----- IMU -----
  Serial5.begin(BNO_UART_BAUD);
  Serial5.setTimeout(0);
  if (bno08x.begin_UART(&Serial5)) {
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    Serial.println("BNO08x IMU ready");
  } else {
    Serial.println("WARNING: BNO08x not found");
  }

  // ----- GPS -----
  gpsSerial.begin(115200);
  Serial.println("GPS passthrough ready (Serial6 @ 115200)");

  // ----- TOF -----
  if (tof.begin()) {
    tofReady = true;
    tof.startRangingContinuous(100);  // 100 ms measurement budget
    Serial.println("VL53L0X TOF ready");
  } else {
    Serial.println("WARNING: VL53L0X not found — TOF disabled");
  }
}

void loop() {
  // ----- Accept new client -----
  if (!client || !client.connected()) {
    EthernetClient incoming = server.available();
    if (incoming) {
      if (client) client.stop();
      client  = incoming;
      cmdLen  = 0;
      client.println("READY");
      Serial.println("Client connected");
    }
  }

  // ----- Read commands -----
  if (client && client.connected()) {
    while (client.available()) {
      char ch = (char)client.read();
      if (ch == '\n' || ch == '\r') {
        if (cmdLen > 0) {
          cmdBuf[cmdLen] = '\0';
          processCommand(cmdBuf);
          cmdLen = 0;
        }
      } else if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = ch;
      }
    }
  }

  // ----- IMU streaming -----
  if (imuStreaming) {
    uint32_t nowMs = millis();
    if (nowMs - lastYprSend >= yprPeriodMs) {
      sh2_SensorValue_t sv;
      if (bno08x.getSensorEvent(&sv) && sv.sensorId == SH2_ROTATION_VECTOR) {
        float yaw, pitch, roll;
        quatToYPR(sv.un.rotationVector.i,
                  sv.un.rotationVector.j,
                  sv.un.rotationVector.k,
                  sv.un.rotationVector.real,
                  yaw, pitch, roll);
        char buf[48];
        snprintf(buf, sizeof(buf), "YPR %.2f %.2f %.2f", yaw, pitch, roll);
        clientPrintln(buf);
      }
      lastYprSend = nowMs;
    }
  }

  // ----- TOF ranging -----
  if (tofReady) {
    uint32_t nowMs = millis();
    if (nowMs - lastTofSend >= TOF_PERIOD_MS) {
      VL53L0X_RangingMeasurementData_t measure;
      tof.getRangingMeasurement(&measure, false);
      char buf[24];
      if (measure.RangeStatus != 4) {
        // Valid reading — convert mm to metres
        float dist_m = measure.RangeMilliMeter / 1000.0f;
        snprintf(buf, sizeof(buf), "TOF %.3f", dist_m);
      } else {
        // Out of range / phase error
        snprintf(buf, sizeof(buf), "TOF -1");
      }
      clientPrintln(buf);
      lastTofSend = nowMs;
    }
  }

  // ----- GPS NMEA passthrough -----
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (nmeaLen < (int)sizeof(nmeaBuffer) - 1) nmeaBuffer[nmeaLen++] = c;
    if (c == '\n') {
      nmeaBuffer[nmeaLen] = '\0';
      if (client && client.connected() && client.availableForWrite() >= nmeaLen)
        client.print(nmeaBuffer);
      nmeaLen = 0;
    }
  }
}
