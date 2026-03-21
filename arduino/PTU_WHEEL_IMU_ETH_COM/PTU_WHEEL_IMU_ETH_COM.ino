#include <AccelStepper.h>
#include <Encoder.h>
#include <NativeEthernet.h>
//#include "Adafruit_BNO08x.h"
#include <math.h>

// ------- WHEEL CONFIGURATION -------
struct Wheel {
  const char* name; // "L" or "R"
  AccelStepper stepper;
  Encoder encoder;
  int stepPin, dirPin, encAPin, encBPin;
  float enc_cpr, steps_per_rev, microsteps, wheel_diam_m, wheel_circ_m;
  float targetSpeed = 0.0f, currentSpeed = 0.0f;
  bool telemetry_on = true;
  volatile long enc_last = 0;
  unsigned long last_enc_us = 0;
  float meas_sps = 0.0f, meas_rpm = 0.0f, meas_ms = 0.0f;
  float filt_sps = 0.0f, filt_rpm = 0.0f, filt_ms = 0.0f;  // per-wheel EMA filters
  unsigned long lastUs = 0;
  unsigned long last_pub = 0;
  char lineBuf[64];
  size_t lineLen = 0;
  Wheel(const char* n, int s, int d, int ea, int eb, float cpr, float spr, float ms, float diam)
    : name(n), stepper(AccelStepper::DRIVER, s, d), encoder(ea, eb),
      stepPin(s), dirPin(d), encAPin(ea), encBPin(eb), enc_cpr(cpr), steps_per_rev(spr), microsteps(ms),
      wheel_diam_m(diam), wheel_circ_m(diam * 3.1415926535f) {}
};

// Pinouts for left and right wheels (adjust as needed)
Wheel leftWheel ("L", 40, 38, 33, 34, 4096.0f, 200.0f, 4.0f, 0.10f);
Wheel rightWheel("R", 39, 41, 35, 36, 4096.0f, 200.0f, 4.0f, 0.10f);

// ------- PTU (Pan/Tilt) CONFIG -------
const int PAN_STEP_PIN   = 4,  PAN_DIR_PIN   = 3;
const int TILT_STEP_PIN  = 6,  TILT_DIR_PIN  = 5;
const int PAN_MS1_PIN = 7, PAN_MS2_PIN = 8, PAN_MS3_PIN = 9;
const int TILT_MS1_PIN = 10, TILT_MS2_PIN = 11, TILT_MS3_PIN = 12;
AccelStepper panMotor (AccelStepper::DRIVER, PAN_STEP_PIN,  PAN_DIR_PIN);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// ------- IMU -------
//Adafruit_BNO08x bno08x;
//const uint32_t BNO_UART_BAUD = 115200;
//bool imuStreaming = false;
//uint32_t yprPeriodMs = 20;
//uint32_t lastYprSend = 0;

// ------- PTU MODE -------
bool  velocityMode     = false;
float panCurrentSpeed  = 0.0f;
float tiltCurrentSpeed = 0.0f;

// ------- Ethernet -------
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 177);
EthernetServer server(23);
EthernetClient client;
String command;

// Helper for reversing (per wheel)
static inline void dirGuardIfReversing(float newS, float oldS) {
  if ((newS > 0 && oldS < 0) || (newS < 0 && oldS > 0)) {
    delayMicroseconds(10);
  }
}

// Dynamic acceleration profile
float getAccelForSpeed(float speed) {
    float absSpeed = fabsf(speed);
    float minAccel = 12000.0f;     // <-- doubled
    float maxAccel = 24000.0f;    // <-- doubled
    float maxSpeed = 25000.0f;
    if (absSpeed >= maxSpeed) return minAccel;
    return maxAccel - (absSpeed / maxSpeed) * (maxAccel - minAccel);
}


// Quaternion → YPR
//static void quatToYPR(float qx, float qy, float qz, float qw,
        //              float &yaw, float &pitch, float &roll) {
//  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
 // float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
//  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

 // float sinp = 2.0f * (qw * qy - qz * qx);
 // pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * 180.0f / PI;

 // float siny_cosp = 2.0f * (qw * qz + qx * qy);
 // float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
 // yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
//}

// ----- Command parsing helpers -----
void processPTUIMUCommand(const String& cmd, Print& c) {
  if (cmd.startsWith("VP") && cmd.indexOf('T') != -1) {
    velocityMode = true;
    String subPan  = cmd.substring(2, cmd.indexOf('T'));
    String subTilt = cmd.substring(cmd.indexOf('T') + 1);
    panCurrentSpeed  = subPan.toFloat();
    tiltCurrentSpeed = subTilt.toFloat();
    panMotor.setSpeed(panCurrentSpeed);
    tiltMotor.setSpeed(tiltCurrentSpeed);
    c.print("C VelocityMode PanSpeed="); c.print(panCurrentSpeed);
    c.print(" TiltSpeed="); c.println(tiltCurrentSpeed);
    return;
  }
  if (cmd.startsWith("P") && cmd.indexOf('T') != -1) {
    velocityMode = false;
    String panString  = cmd.substring(1, cmd.indexOf('T'));
    String tiltString = cmd.substring(cmd.indexOf('T') + 1);
    long panValue  = panString.toInt();
    long tiltValue = tiltString.toInt();
    panMotor.moveTo(panValue);
    tiltMotor.moveTo(tiltValue);
    return;
  }
 // if (cmd.equalsIgnoreCase("IMUON"))  { imuStreaming = true;  c.println("C IMU Streaming ON");  return; }
  //if (cmd.equalsIgnoreCase("IMUOFF")) { imuStreaming = false; c.println("C IMU Streaming OFF"); return; }
 // if (cmd.startsWith("IMURATE")) {
  //  int sp = cmd.indexOf(' ');
  //  if (sp > 0) {
    //  float hz = cmd.substring(sp + 1).toFloat();
   //   if (hz < 1) hz = 1;
   //   if (hz > 200) hz = 200;
   //   yprPeriodMs = (uint32_t)(1000.0f / hz);
   //   c.print("C IMU Rate = "); c.print(hz, 1); c.println(" Hz");
   //   return;
  //  }
 // }
  c.println("ERR Unrecognized Command");
}

// Unified command parser
void processUnifiedCommand(const String& cmd, EthernetClient* c = nullptr) {
  const char* s = cmd.c_str();
  while (*s) {
    // Skip whitespace
    while (*s && isspace(*s)) s++;

    // LS/RS
    if (strncmp(s, "LS", 2) == 0) {
      s += 2;
      while (*s && isspace(*s)) s++;
      float v = 0;
      int chars = 0;
      if (sscanf(s, "%f%n", &v, &chars) == 1) {
        leftWheel.targetSpeed = (fabs(v) > leftWheel.stepper.maxSpeed()) ? copysign(leftWheel.stepper.maxSpeed(), v) : v;
        String msg = "L New target speed: " + String(leftWheel.targetSpeed, 1);
        if (c) c->println(msg); else Serial.println(msg);
        s += chars;
        continue;
      }
    }
    if (strncmp(s, "RS", 2) == 0) {
      s += 2;
      while (*s && isspace(*s)) s++;
      float v = 0;
      int chars = 0;
      if (sscanf(s, "%f%n", &v, &chars) == 1) {
        rightWheel.targetSpeed = (fabs(v) > rightWheel.stepper.maxSpeed()) ? copysign(rightWheel.stepper.maxSpeed(), v) : v;
        String msg = "R New target speed: " + String(rightWheel.targetSpeed, 1);
        if (c) c->println(msg); else Serial.println(msg);
        s += chars;
        continue;
      }
    }
    // LT/RT
    if (strncmp(s, "LT", 2) == 0) {
      s += 2;
      while (*s && isspace(*s)) s++;
      int t = (*s == '1') ? 1 : 0;
      leftWheel.telemetry_on = t;
      String msg = "L Telemetry " + String(leftWheel.telemetry_on ? "ON" : "OFF");
      if (c) c->println(msg); else Serial.println(msg);
      s++;
      continue;
    }
    if (strncmp(s, "RT", 2) == 0) {
      s += 2;
      while (*s && isspace(*s)) s++;
      int t = (*s == '1') ? 1 : 0;
      rightWheel.telemetry_on = t;
      String msg = "R Telemetry " + String(rightWheel.telemetry_on ? "ON" : "OFF");
      if (c) c->println(msg); else Serial.println(msg);
      s++;
      continue;
    }
    // Legacy S for left only
    if (*s == 'S') {
      s++;
      while (*s && isspace(*s)) s++;
      float v = 0;
      int chars = 0;
      if (sscanf(s, "%f%n", &v, &chars) == 1) {
        leftWheel.targetSpeed = (fabs(v) > leftWheel.stepper.maxSpeed()) ? copysign(leftWheel.stepper.maxSpeed(), v) : v;
        String msg = "L New target speed: " + String(leftWheel.targetSpeed, 1);
        if (c) c->println(msg); else Serial.println(msg);
        s += chars;
        continue;
      }
    }
    // Legacy T for left only
    if (*s == 'T') {
      s++;
      while (*s && isspace(*s)) s++;
      int t = (*s == '1') ? 1 : 0;
      leftWheel.telemetry_on = t;
      String msg = "L Telemetry " + String(leftWheel.telemetry_on ? "ON" : "OFF");
      if (c) c->println(msg); else Serial.println(msg);
      s++;
      continue;
    }
    // PTU/IMU or unknown: pass the rest
    String rest = String(s);
    rest.trim();
    if (rest.length() > 0) {
      if (c) processPTUIMUCommand(rest, *c);
      else processPTUIMUCommand(rest, Serial);
    }
    break;
  }
}

// PTU/IMU streaming to Print (Serial or Ethernet)
//void streamIMU(Print& out) {
 // sh2_SensorValue_t sv;
//  if (bno08x.getSensorEvent(&sv) && sv.sensorId == SH2_ROTATION_VECTOR) {
  //  float yaw, pitch, roll;
  //  quatToYPR(sv.un.rotationVector.i, sv.un.rotationVector.j, sv.un.rotationVector.k, sv.un.rotationVector.real, yaw, pitch, roll);
  //  out.print("YPR ");
   // out.print(yaw, 2); out.print(" ");
  //  out.print(pitch, 2); out.print(" ");
   // out.println(roll, 2);
  //}
//}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  leftWheel.lastUs = micros(); leftWheel.last_enc_us = leftWheel.lastUs; leftWheel.enc_last = leftWheel.encoder.read();
  rightWheel.lastUs = micros(); rightWheel.last_enc_us = rightWheel.lastUs; rightWheel.enc_last = rightWheel.encoder.read();
  Serial.println("Wheels: 'LS<spd>RS<spd>', 'LT<0|1>RT<0|1>', legacy 'S','T' for left. PTU/IMU: VP,P,IMUON,IMUOFF,IMURATE.");

  // Important: Set max speed/min pulse width for wheel steppers!
  leftWheel.stepper.setMaxSpeed(20000);
  leftWheel.stepper.setMinPulseWidth(8);
  rightWheel.stepper.setMaxSpeed(20000);
  rightWheel.stepper.setMinPulseWidth(8);

  pinMode(PAN_STEP_PIN,  OUTPUT); pinMode(PAN_DIR_PIN,   OUTPUT);
  pinMode(TILT_STEP_PIN, OUTPUT); pinMode(TILT_DIR_PIN,  OUTPUT);
  pinMode(PAN_MS1_PIN, OUTPUT); pinMode(PAN_MS2_PIN, OUTPUT); pinMode(PAN_MS3_PIN, OUTPUT);
  pinMode(TILT_MS1_PIN, OUTPUT); pinMode(TILT_MS2_PIN, OUTPUT); pinMode(TILT_MS3_PIN, OUTPUT);
  digitalWrite(PAN_MS1_PIN, HIGH); digitalWrite(PAN_MS2_PIN, HIGH); digitalWrite(PAN_MS3_PIN, LOW);
  digitalWrite(TILT_MS1_PIN, HIGH); digitalWrite(TILT_MS2_PIN, HIGH); digitalWrite(TILT_MS3_PIN, LOW);
  panMotor.setMaxSpeed(12000); panMotor.setAcceleration(15000); panMotor.setMinPulseWidth(1);
  tiltMotor.setMaxSpeed(12000); tiltMotor.setAcceleration(15000); tiltMotor.setMinPulseWidth(1);

  Ethernet.begin(mac, ip); server.begin();

 // Serial5.begin(BNO_UART_BAUD);
  //if (bno08x.begin_UART(&Serial5)) { bno08x.enableReport(SH2_ROTATION_VECTOR); }
}

// ----- Wheel ramp/telemetry logic -----
void wheelLoop(Wheel& wheel, bool isSerial) {
  unsigned long now = micros();
  float dt = (now - wheel.lastUs) / 1e6f;
  wheel.lastUs = now;

  // Use higher of current/target speed for torque-aware ramping
  float speedForAccel = fmaxf(fabsf(wheel.currentSpeed), fabsf(wheel.targetSpeed));
  float accel = getAccelForSpeed(speedForAccel);
  float delta = accel * dt;

  float prev  = wheel.currentSpeed;
  if (wheel.currentSpeed < wheel.targetSpeed)
      wheel.currentSpeed = fminf(wheel.currentSpeed + delta, wheel.targetSpeed);
  else if (wheel.currentSpeed > wheel.targetSpeed)
      wheel.currentSpeed = fmaxf(wheel.currentSpeed - delta, wheel.targetSpeed);

  dirGuardIfReversing(wheel.currentSpeed, prev);
  wheel.stepper.setSpeed(wheel.currentSpeed);
  wheel.stepper.runSpeed();

  // --- encoder telemetry every 50 ms ---
  if ((now - wheel.last_pub) >= 50000) {
    long enc_now = wheel.encoder.read();
    long dcounts = enc_now - wheel.enc_last;
    unsigned long enc_now_us = now;
    float dt_enc = (enc_now_us - wheel.last_enc_us) / 1e6f;
    wheel.enc_last = enc_now;
    wheel.last_enc_us = enc_now_us;
    wheel.last_pub = now;
    float rev_s = (dt_enc > 0) ? (dcounts / wheel.enc_cpr) / dt_enc : 0.0f;
    wheel.meas_rpm = rev_s * 60.0f;
    wheel.meas_ms  = rev_s * wheel.wheel_circ_m;
    wheel.meas_sps = rev_s * (wheel.steps_per_rev * wheel.microsteps);
    const float alpha = 0.3f;
    wheel.filt_rpm = (1 - alpha) * wheel.filt_rpm + alpha * wheel.meas_rpm;
    wheel.filt_ms  = (1 - alpha) * wheel.filt_ms  + alpha * wheel.meas_ms;
    wheel.filt_sps = (1 - alpha) * wheel.filt_sps + alpha * wheel.meas_sps;
    if (wheel.telemetry_on && isSerial) {
      // Verbose per-wheel debug output on Serial only
      float time_s = millis() / 1000.0f;
      String line = String(wheel.name) + "," + String(time_s, 3) + "," +
        String(wheel.targetSpeed, 1) + "," +
        String(wheel.currentSpeed, 1) + "," +
        String(wheel.filt_sps, 1) + "," +
        String(wheel.filt_rpm, 2) + "," +
        String(wheel.filt_ms, 3);
      Serial.println(line);
    }
  }
}

// ----- Main loop -----
void loop() {
  // ------------- SERIAL COMMAND INPUT (line-based, supports both) -------------
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (leftWheel.lineLen > 0) {
        leftWheel.lineBuf[leftWheel.lineLen] = '\0';
        processUnifiedCommand(String(leftWheel.lineBuf), nullptr);
        leftWheel.lineLen = 0;
      }
    } else if (leftWheel.lineLen < sizeof(leftWheel.lineBuf) - 1) {
      leftWheel.lineBuf[leftWheel.lineLen++] = c;
    }
  }
  // ------------- ETHERNET COMMAND INPUT (line-based, supports both) -----------
  if (!client || !client.connected()) {
    EthernetClient incoming = server.available();
    if (incoming) {
      if (client) client.stop();
      client = incoming;
      client.println("READY");
      command = "";
    }
  }
  if (client && client.connected()) {
    while (client.available()) {
      char ch = client.read();
      if (ch == '\n' || ch == '\r') {
        if (command.length() > 0) {
          command.trim();
          Serial.print("TCP RX: "); Serial.println(command); // Print TCP command to serial monitor
          processUnifiedCommand(command, &client);
          command = "";
        }
      } else {
        command += ch;
      }
    }
  }

  // ----------- WHEEL/TELEMETRY LOGIC (Serial & Ethernet) -----------
  wheelLoop(leftWheel,  true);   // updates filt_sps, prints verbose to Serial
  wheelLoop(rightWheel, true);

  // Send combined encoder line to Jetson every 50 ms — format: "ENC <left_sps> <right_sps>"
  static unsigned long lastEncPub = 0;
  unsigned long nowMs = millis();
  if (client && client.connected() && (nowMs - lastEncPub >= 50)) {
    char encBuf[32];
    snprintf(encBuf, sizeof(encBuf), "ENC %d %d",
             (int)leftWheel.filt_sps, (int)rightWheel.filt_sps);
    client.println(encBuf);
    lastEncPub = nowMs;
  }

  // ----------- PTU CONTROL -----------
  if (velocityMode) {
    panMotor.setSpeed(panCurrentSpeed);
    tiltMotor.setSpeed(tiltCurrentSpeed);
    panMotor.runSpeed();
    tiltMotor.runSpeed();
  } else {
    panMotor.run();
    tiltMotor.run();
  }

  // ----------- IMU STREAMING -----------
 // if (imuStreaming) {
 //   uint32_t nowMs = millis();
  //  if (nowMs - lastYprSend >= yprPeriodMs) {
      // Stream to Serial
  //    streamIMU(Serial);
      // Stream to Ethernet (if connected)
  //    if (client && client.connected() && client.availableForWrite() >= 40) {
  //      streamIMU(client);
  //    }
  //    lastYprSend = nowMs;
  //  }
 // }
}