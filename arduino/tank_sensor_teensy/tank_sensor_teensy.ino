/*
 * SENSOR_TEENSY.ino — Tank C2 dedicated sensor node
 * Hardened: BNO085 body IMU, BN880 GPS, TOF, wheel encoders, NativeEthernet.
 *
 * Hardware (Teensy 4.1):
 *
 *   BNO085 Body IMU   SH2-UART on Serial3  TX3=pin14 → IMU RX
 *                                           RX3=pin15 ← IMU TX
 *                     PS0=GND, PS1=3V3  (SH2-UART mode, 115200 baud)
 *
 *   BN880 GPS         Serial6  RX6=pin25 ← GPS TX,  TX6=pin24 → GPS RX
 *                     115200 baud (module pre-configured), raw NMEA forwarded.
 *
 *   TOF Sensor        Serial7  RX7=pin28 ← TOF TX,  TX7=pin29 → TOF RX
 *                     921600 baud, 16-byte binary frame (0x57 header).
 *
 *   Left  Encoder     A=pin33 (interrupt), B=pin34 (direction)
 *   Right Encoder     A=pin36 (interrupt), B=pin35 (direction)
 *
 *   Ethernet          Built-in Teensy 4.1 ENET.
 *                     Static IP 192.168.1.178, TCP port 23.
 *
 * ── Outbound protocol (Teensy → Jetson) ─────────────────────────────────────
 *   SENSOR_READY                          on each new client connection
 *   YPR <yaw> <pitch> <roll>             body IMU (degrees, at imuRate Hz)
 *   $GNGGA,...  /  $GNRMC,...            validated raw NMEA (always on)
 *   ENC <left_sps> <right_sps>           encoder steps/second (always on)
 *   TOF <dist_m>                         TOF distance in metres (always on)
 *
 * ── Inbound protocol (Jetson → Teensy) ──────────────────────────────────────
 *   IMU_ON            enable YPR streaming (auto-enabled on connect)
 *   IMU_OFF           disable YPR streaming
 *   IMU_RATE <hz>     set YPR send rate 1–200 Hz (default 50)
 *
 * ── Required libraries ───────────────────────────────────────────────────────
 *   Adafruit BNO08x     Tools → Manage Libraries → "Adafruit BNO08x"
 *   NativeEthernet      bundled with Teensyduino
 *
 * ── Optional hardware watchdog ───────────────────────────────────────────────
 *   Install WDT_T4 from github.com/tonton81/WDT_T4, then uncomment USE_WATCHDOG.
 */

// #define USE_WATCHDOG

#ifdef USE_WATCHDOG
#include <Watchdog_t4.h>
WDT_T4<WDT1> wdt;
#endif

// Adafruit BNO08x defines PI internally; prevent conflict with math.h
#ifdef PI
#undef PI
#endif
#define PI 3.14159265358979323846f

#include "Adafruit_BNO08x.h"
#include <NativeEthernet.h>
#include <math.h>

// ── Tunables ─────────────────────────────────────────────────────────────────
static const uint32_t IMU_STALE_TIMEOUT_MS   = 5000;   // mark IMU dead after 5 s silence
static const uint32_t ENC_REPORT_INTERVAL_MS = 100;    // encoder SPS report rate (10 Hz)
static const uint32_t SEND_STALL_TIMEOUT_MS  = 3000;   // drop client on TX stall
static const uint32_t STATUS_INTERVAL_MS     = 10000;  // USB-serial status interval
static const int      MIN_TX_FREE            = 80;     // min TX buffer bytes before send
static const int      MAX_GPS_PER_LOOP       = 96;     // GPS bytes consumed per loop tick
static const uint32_t WDT_TIMEOUT_S          = 6;

// ── Serial port assignments ───────────────────────────────────────────────────
#define IMU_SERIAL   Serial3   // TX3=pin14, RX3=pin15
#define GPS_SERIAL   Serial6   // TX6=pin24, RX6=pin25
#define TOF_SERIAL   Serial7   // TX7=pin29, RX7=pin28

static const uint32_t GPS_BAUD = 115200;
static const uint32_t TOF_BAUD = 921600;

// ── Encoder pins ─────────────────────────────────────────────────────────────
static const int LEFT_ENC_A  = 33;
static const int LEFT_ENC_B  = 34;
static const int RIGHT_ENC_A = 36;
static const int RIGHT_ENC_B = 35;

// ── Network ───────────────────────────────────────────────────────────────────
static byte          mac[]     = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF };
static const IPAddress SENSOR_IP(192, 168, 1, 178);
static const uint16_t  SENSOR_PORT = 23;

EthernetServer server(SENSOR_PORT);
EthernetClient client;
static bool     clientActive     = false;
static uint32_t sendStallStartMs = 0;

// ── IMU ───────────────────────────────────────────────────────────────────────
Adafruit_BNO08x  imu(-1);   // -1 = no hardware reset pin
static bool      imuOk         = false;
static uint32_t  lastImuDataMs = 0;
static bool      imuStreaming  = false;
static uint32_t  imuPeriodUs   = 20000;   // 50 Hz default send rate
static uint32_t  lastImuSendUs = 0;
static float     dbgYaw = NAN, dbgPitch = NAN, dbgRoll = NAN;
static char      dbgGpsLine[80] = {};

static void enableImuReports() {
    imu.enableReport(SH2_ROTATION_VECTOR, 20);
}

// ── Quaternion → YPR (degrees) ────────────────────────────────────────────────
static void quatToYPR(float qx, float qy, float qz, float qw,
                      float& yaw, float& pitch, float& roll) {
    float sr = 2.0f*(qw*qx + qy*qz), cr = 1.0f - 2.0f*(qx*qx + qy*qy);
    roll  = atan2f(sr, cr) * 180.0f / PI;
    float sp = 2.0f*(qw*qy - qz*qx);
    pitch = (fabsf(sp) >= 1.0f) ? copysignf(90.0f, sp) : asinf(sp) * 180.0f / PI;
    float sy = 2.0f*(qw*qz + qx*qy), cy = 1.0f - 2.0f*(qy*qy + qz*qz);
    yaw   = atan2f(sy, cy) * 180.0f / PI;
}

// ── Encoders ──────────────────────────────────────────────────────────────────
static volatile int32_t leftEncCount  = 0;
static volatile int32_t rightEncCount = 0;

static void leftEncISR()  { if (digitalRead(LEFT_ENC_B))  leftEncCount++; else leftEncCount--; }
static void rightEncISR() { if (digitalRead(RIGHT_ENC_B)) rightEncCount++; else rightEncCount--; }

// ── TOF sensor ────────────────────────────────────────────────────────────────
static uint8_t tofBuf[16];
static int     tofBufLen = 0;

// ── GPS / command buffers ─────────────────────────────────────────────────────
static char gpsBuf[160];
static int  gpsLen = 0;
static char cmdBuf[160];
static int  cmdLen = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Watchdog
// ─────────────────────────────────────────────────────────────────────────────
static inline void wdFeed() {
#ifdef USE_WATCHDOG
    wdt.feed();
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
//  TOF byte accumulator — returns metres on a valid frame, else -1
// ─────────────────────────────────────────────────────────────────────────────
static float processTofByte(uint8_t b) {
    if (tofBufLen == 0 && b != 0x57) return -1.0f;
    tofBuf[tofBufLen++] = b;
    if (tofBufLen < 16) return -1.0f;
    tofBufLen = 0;
    uint8_t sum = 0;
    for (int i = 0; i < 15; i++) sum += tofBuf[i];
    if (sum != tofBuf[15]) return -1.0f;
    if (tofBuf[11] != 1)   return -1.0f;
    uint32_t mm = (uint32_t)tofBuf[8] | ((uint32_t)tofBuf[9]<<8) | ((uint32_t)tofBuf[10]<<16);
    return mm / 1000.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  NMEA checksum validation
// ─────────────────────────────────────────────────────────────────────────────
static bool validateNmea(const char* s, int len) {
    if (len < 7 || s[0] != '$') return false;
    int star = -1;
    for (int i = 1; i < len; i++) { if (s[i] == '*') { star = i; break; } }
    if (star < 0 || star + 2 >= len) return false;
    uint8_t cs = 0;
    for (int i = 1; i < star; i++) cs ^= (uint8_t)s[i];
    auto hexVal = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    int hi = hexVal(s[star+1]), lo = hexVal(s[star+2]);
    if (hi < 0 || lo < 0) return false;
    return cs == (uint8_t)((hi << 4) | lo);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Ethernet helpers
// ─────────────────────────────────────────────────────────────────────────────
static void onDisconnect() {
    if (clientActive) Serial.println("[NET] Client disconnected");
    clientActive = false; imuStreaming = false;
    sendStallStartMs = 0; cmdLen = 0; gpsLen = 0;
    if (client) client.stop();
}

static void onConnect(EthernetClient& nc) {
    onDisconnect();
    client = nc; clientActive = true; imuStreaming = true;
    client.println("SENSOR_READY");
    Serial.println("[NET] Jetson connected");
}

static bool safeSend(const char* line) {
    if (!clientActive || !client || !client.connected()) return false;
    int avail = client.availableForWrite();
    if (avail < MIN_TX_FREE) {
        if (sendStallStartMs == 0) sendStallStartMs = millis();
        if (millis() - sendStallStartMs > SEND_STALL_TIMEOUT_MS) {
            Serial.println("[NET] TX stall — dropping client");
            onDisconnect();
        }
        return false;
    }
    sendStallStartMs = 0;
    if (!client.println(line)) { onDisconnect(); return false; }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Command parser
// ─────────────────────────────────────────────────────────────────────────────
static void processCommand(const char* raw) {
    char cmd[160]; int len = 0;
    const char* p = raw;
    while (*p == ' ' || *p == '\t') p++;
    while (*p && len < (int)sizeof(cmd)-1) cmd[len++] = *p++;
    cmd[len] = '\0';
    while (len > 0 && (cmd[len-1]==' '||cmd[len-1]=='\t'||cmd[len-1]=='\r')) cmd[--len]='\0';
    if (len == 0) return;

    const char* c = (strncmp(cmd, "IMU ", 4) == 0) ? cmd+4 : cmd;

    if (strcasecmp(c, "IMU_ON") == 0 || strcasecmp(c, "IMUON") == 0) {
        imuStreaming = true; safeSend("C IMU Streaming ON"); return;
    }
    if (strcasecmp(c, "IMU_OFF") == 0 || strcasecmp(c, "IMUOFF") == 0) {
        imuStreaming = false; safeSend("C IMU Streaming OFF"); return;
    }
    if (strncasecmp(c, "IMU_RATE", 8) == 0 && (c[8]==' '||c[8]=='\0')) {
        float hz = (c[8]==' ') ? (float)atof(c+9) : 50.0f;
        if (hz < 1.0f) hz = 1.0f; if (hz > 200.0f) hz = 200.0f;
        imuPeriodUs = (uint32_t)(1000000.0f / hz);
        char msg[48]; snprintf(msg, sizeof(msg), "C IMU Rate = %.1f Hz", hz);
        safeSend(msg); return;
    }
    char err[80]; snprintf(err, sizeof(err), "ERR Unknown: %.60s", c);
    safeSend(err);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println("\n[SENSOR_TEENSY] Booting...");

#ifdef USE_WATCHDOG
    WDT_timings_t cfg; cfg.timeout = WDT_TIMEOUT_S; wdt.begin(cfg);
    Serial.printf("[WDT] Watchdog enabled, timeout = %lu s\n", WDT_TIMEOUT_S);
#endif

    // ── Ethernet FIRST — claims ENET DMA before UART DMA ───────────────────
    Ethernet.begin(mac, SENSOR_IP);
    server.begin();
    Serial.printf("[NET] Listening on %d.%d.%d.%d:%d\n",
        SENSOR_IP[0], SENSOR_IP[1], SENSOR_IP[2], SENSOR_IP[3], SENSOR_PORT);

    // ── GPS ─────────────────────────────────────────────────────────────────
    GPS_SERIAL.begin(GPS_BAUD);
    Serial.println("[GPS] BN880 on Serial6 (RX=25, TX=24) 115200 baud");

    // ── TOF ─────────────────────────────────────────────────────────────────
    TOF_SERIAL.begin(TOF_BAUD);
    Serial.println("[TOF] Sensor on Serial7 (RX=28, TX=29) 921600 baud");

    // ── Encoders ────────────────────────────────────────────────────────────
    pinMode(LEFT_ENC_A,  INPUT_PULLUP); pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR, CHANGE);
    Serial.println("[ENC] Left A=33 B=34 / Right A=36 B=35");

    // ── IMU — after Ethernet ─────────────────────────────────────────────────
    IMU_SERIAL.begin(115200);
    IMU_SERIAL.setTimeout(0);
    if (imu.begin_UART(&IMU_SERIAL)) {
        enableImuReports();
        imuOk         = true;
        lastImuDataMs = millis();
        Serial.println("[IMU] Body IMU (Serial3) OK");
    } else {
        Serial.println("[IMU] Body IMU (Serial3) FAIL — check wiring / PS0=GND PS1=3V3");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main loop — fully non-blocking
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    const uint32_t now = millis();

    wdFeed();

    // ── Ethernet link monitoring ──────────────────────────────────────────
    {
        static EthernetLinkStatus lastLink = Unknown;
        EthernetLinkStatus link = Ethernet.linkStatus();
        if (link != lastLink) {
            if      (link == LinkOFF) { Serial.println("[NET] Link DOWN"); onDisconnect(); }
            else if (link == LinkON)  { Serial.println("[NET] Link UP"); }
            lastLink = link;
        }
    }

    // ── Accept new client / detect remote close ───────────────────────────
    if (!clientActive || !client || !client.connected()) {
        if (clientActive) onDisconnect();
        EthernetClient inc = server.available();
        if (inc) onConnect(inc);
    }

    // ── Receive and parse commands ────────────────────────────────────────
    if (clientActive && client.connected()) {
        while (client.available()) {
            char ch = (char)client.read();
            if (ch == '\n' || ch == '\r') {
                if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; processCommand(cmdBuf); cmdLen = 0; }
            } else if (cmdLen < (int)sizeof(cmdBuf)-1) {
                cmdBuf[cmdLen++] = ch;
            } else { cmdLen = 0; }
        }
    }

    // ── Body IMU ──────────────────────────────────────────────────────────
    if (imuOk) {
        if (imu.wasReset()) {
            Serial.println("[IMU] Reset detected — re-enabling reports");
            enableImuReports();
        }
        sh2_SensorValue_t sv;
        if (imu.getSensorEvent(&sv) && sv.sensorId == SH2_ROTATION_VECTOR) {
            lastImuDataMs = now;
            float yaw, pitch, roll;
            quatToYPR(sv.un.rotationVector.i,  sv.un.rotationVector.j,
                      sv.un.rotationVector.k,  sv.un.rotationVector.real,
                      yaw, pitch, roll);
            dbgYaw = yaw; dbgPitch = pitch; dbgRoll = roll;
            if (imuStreaming) {
                uint32_t us = micros();
                if (us - lastImuSendUs >= imuPeriodUs) {
                    lastImuSendUs = us;
                    char buf[48];
                    snprintf(buf, sizeof(buf), "YPR %.2f %.2f %.2f", yaw, pitch, roll);
                    safeSend(buf);
                }
            }
        }
        // Stale detection — report only, no reinit (begin_UART blocks)
        if (lastImuDataMs > 0 && (now - lastImuDataMs > IMU_STALE_TIMEOUT_MS)) {
            Serial.println("[IMU] WARNING: no data for 5 s");
            lastImuDataMs = now;  // suppress repeated warnings
            imuOk = false;        // status shows ERR — reboot Teensy to recover
        }
    }

    // ── Encoder SPS — always sent when client connected ───────────────────
    {
        static uint32_t lastEncMs = 0;
        static int32_t  prevL = 0, prevR = 0;
        if (now - lastEncMs >= ENC_REPORT_INTERVAL_MS) {
            uint32_t dt = now - lastEncMs;  lastEncMs = now;
            int32_t curL = leftEncCount, curR = rightEncCount;
            int32_t dL = curL - prevL, dR = curR - prevR;
            prevL = curL; prevR = curR;
            if (clientActive) {
                int32_t spsL = (int32_t)((float)dL / (float)dt * 1000.0f);
                int32_t spsR = (int32_t)((float)dR / (float)dt * 1000.0f);
                char buf[40];
                snprintf(buf, sizeof(buf), "ENC %ld %ld", (long)spsL, (long)spsR);
                safeSend(buf);
            }
        }
    }

    // ── TOF sensor — always sent when client connected ────────────────────
    while (TOF_SERIAL.available()) {
        float dist = processTofByte((uint8_t)TOF_SERIAL.read());
        if (dist >= 0.0f && clientActive) {
            char buf[24]; snprintf(buf, sizeof(buf), "TOF %.3f", dist);
            safeSend(buf);
        }
    }

    // ── GPS — forward validated NMEA ──────────────────────────────────────
    {
        int n = 0;
        while (GPS_SERIAL.available() && n < MAX_GPS_PER_LOOP) {
            ++n;
            char ch = (char)GPS_SERIAL.read();
            if (ch == '\n') {
                if (gpsLen > 0) {
                    gpsBuf[gpsLen] = '\0';
                    if (validateNmea(gpsBuf, gpsLen)) {
                        safeSend(gpsBuf);
                        snprintf(dbgGpsLine, sizeof(dbgGpsLine), "%.79s", gpsBuf);
                    }
                    gpsLen = 0;
                }
            } else if (ch != '\r') {
                if (gpsLen < (int)sizeof(gpsBuf)-1) gpsBuf[gpsLen++] = ch;
                else gpsLen = 0;
            }
        }
    }

    // ── 2s debug: IMU + GPS ───────────────────────────────────────────────
    {
        static uint32_t lastDbgMs = 0;
        if (now - lastDbgMs >= 2000) {
            lastDbgMs = now;
            if (imuOk && !isnan(dbgYaw))
                Serial.printf("[DBG] IMU  Y=%6.1f  P=%6.1f  R=%6.1f\n", dbgYaw, dbgPitch, dbgRoll);
            else
                Serial.println("[DBG] IMU  no data");
            if (dbgGpsLine[0])
                Serial.printf("[DBG] GPS  %s\n", dbgGpsLine);
            else
                Serial.println("[DBG] GPS  no data");
        }
    }

    // ── Periodic USB-serial status ────────────────────────────────────────
    {
        static uint32_t lastStatusMs = 0;
        if (now - lastStatusMs > STATUS_INTERVAL_MS) {
            lastStatusMs = now;
            Serial.printf("[STATUS] imu=%s  stream=%s  link=%s  client=%s\n",
                imuOk        ? "OK"   : "ERR",
                imuStreaming ? "ON"   : "OFF",
                (Ethernet.linkStatus() == LinkON) ? "UP" : "DOWN",
                clientActive ? "CONN" : "wait");
        }
    }
}
