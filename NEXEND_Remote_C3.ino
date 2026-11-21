/*
 NEXEND_Remote_C3.ino  -  Remote NEXEND FC (ESP32-C3 SuperMini)
 Board : "ESP32C3 Dev Module" (core esp32 by Espressif, v3.x)
 USB   : Tools > USB CDC On Boot = "Enabled"  (Serial = USB CDC bawaan C3)
         Flash Mode = DIO, CPU 160 MHz. Wajib Enabled, kalau tidak APK tidak terhubung.
 Library: RF24 (TMRh20) >= 1.4.8

 Jalur : APK NEXEND FLY <-> USB CDC (JSON Lines) <-> remote <-> nRF24L01 <-> NEXEND FC
 Remote TIDAK menjalankan PID/ESC/failsafe penerbangan. Keputusan ARM/RTH/failsafe ada di FC.
 USB juga menerima protokol biner AA 55 (lihat blok [BIN]); mode dipilih otomatis per paket.
 Protokol radio NEXEND_LINK_V1: FC harus membalas lewat ACK payload (lihat dokumen pendamping).
 Status kompatibilitas penuh dengan FC/APK BELUM terbukti sebelum diuji bersama.
*/
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stddef.h>

// ======================= KONSTANTA =======================
#define FW_MAJ 1
#define FW_MIN 1
#define FW_STR "1.1.0"   // [RA] 1.1.0: SET/GET_RTH_ALT
#define PROTO_NAME "NEXEND_LINK_V1"
#define PROTO_VER 1
#define PKT_MAGIC 0xA7
#define PKT_HDR 6
#define PKT_MAX_PL 24

// ESP32-C3 SuperMini: ADC1 = GPIO0..4 (GPIO5 = ADC2, jangan dipakai). GPIO2/8/9 = strapping, dihindari.
// GPIO18/19 = USB, jangan dipakai.
static const uint8_t PIN_JOY[4] = {0, 1, 3, 4}; // LX, LY, RX, RY
#define PIN_NRF_CE 10
#define PIN_NRF_CSN 20
#define PIN_NRF_SCK 6
#define PIN_NRF_MOSI 7
#define PIN_NRF_MISO 5
#define PIN_RTH 21   // tombol RTH ke GND. IRQ nRF tidak dipakai: radio di-poll

#define JOY_PERIOD_MS 5
#define RAW_MIN_OK 30
#define RAW_MAX_OK 4095   // ADC C3 jenuh sebelum 3.3V; deteksi 'nyangkut atas' dimatikan, deteksi 'nyangkut bawah' tetap
#define DEBOUNCE_MS 30
#define NEUTRAL_ARM_MS 500
#define NEUTRAL_FLIGHT_MS 300
#define ARM_PENDING_MS 2500
#define USB_ACTIVE_MS 5000
#define LINE_MAX 256
#define NSLOT 6
#define CAL_MIN_SPAN 500
#define CAL_TIMEOUT_MS 120000UL
#define PID_MAX 100.0f

#define EE_CAL_ADDR 0
#define EE_CFG_ADDR 96
#define EE_MAGIC_CAL 0x4E584331UL
#define EE_MAGIC_CFG 0x4E584332UL

// ======================= TIPE =======================
enum RemoteState : uint8_t { RS_CALIBRATION_REQUIRED, RS_CALIBRATING, RS_DISARMED, RS_ARM_GESTURE, RS_ARM_PENDING, RS_ARMED_IDLE, RS_FLIGHT };
enum LinkState : uint8_t { LS_NOT_INIT, LS_SEARCHING, LS_CONNECTED, LS_LOST };
enum CalState : uint8_t { CAL_NONE, CAL_CAPTURE, CAL_SWEEP };
enum FcState : uint8_t { FC_UNKNOWN = 0, FC_DISARMED = 1, FC_ARMING = 2, FC_ARMED = 3, FC_FAILSAFE = 4, FC_ERROR = 5 };

enum PktType : uint8_t {
  PT_HELLO = 0x01, PT_PAIR_STATUS = 0x02, PT_HEARTBEAT = 0x03,
  PT_CONTROL = 0x10, PT_ARM_REQUEST = 0x11, PT_ARM_STATUS = 0x12, PT_RTH_REQUEST = 0x13, PT_RTH_STATUS = 0x14,
  PT_TELEM_BASIC = 0x20, PT_TELEM_GPS = 0x21, PT_TELEM_BATTERY = 0x22, PT_TELEM_SENSORS = 0x23, PT_TELEM_FLIGHT = 0x24, PT_TELEM_LINK = 0x25,
  PT_COMMAND_REQ = 0x30, PT_COMMAND_RESP = 0x31,
  PT_PID_READ_REQ = 0x40, PT_PID_READ_RESP = 0x41, PT_PID_WRITE_REQ = 0x42, PT_PID_WRITE_RESP = 0x43,
  PT_CAL_REQ = 0x44, PT_CAL_STATUS = 0x45, PT_LOG_REQ = 0x46, PT_LOG_RESP = 0x47,
  PT_ERROR = 0x7F
};

enum : uint32_t {
  CAP_ARM = 1UL << 0, CAP_DISARM = 1UL << 1, CAP_RTH = 1UL << 2, CAP_PID_READ = 1UL << 3, CAP_PID_WRITE = 1UL << 4,
  CAP_CALIB = 1UL << 5, CAP_LOG_READ = 1UL << 6, CAP_LOG_CLEAR = 1UL << 7, CAP_HOME = 1UL << 8,
  CAP_T_BATT = 1UL << 9, CAP_T_GPS = 1UL << 10, CAP_T_SENS = 1UL << 11, CAP_T_FLIGHT = 1UL << 12, CAP_T_LINK = 1UL << 13,
  CAP_RTH_ALT = 1UL << 14   // [RA] FC mendukung set/baca ketinggian RTH maksimal
};

enum { T_BASIC = 0, T_BATT, T_GPS, T_SENS, T_FLIGHT, T_LINK, T_COUNT };
enum { SRC_APK = 1, SRC_GESTURE = 2, SRC_BUTTON = 3 };
enum { C_NONE = 0, C_ARM, C_DISARM, C_RTH, C_GET_PID, C_SET_PID, C_CALIB, C_LOG, C_CLEAR_LOG, C_HOME, C_RTHALT };

enum {
  U_NONE = 0, U_HELLO, U_GET_STATUS, U_GET_REMOTE_STATUS, U_GET_FC_STATUS, U_GET_TELEMETRY, U_GET_BATTERY, U_GET_GPS,
  U_GET_FLIGHT_DATA, U_GET_SENSOR_STATUS, U_GET_LINK_STATS, U_GET_HOME_POINT, U_GET_FLIGHT_MODE, U_GET_PID, U_SET_PID,
  U_CALIBRATE_REQUEST, U_GET_FLIGHT_LOGS, U_CLEAR_FLIGHT_LOGS, U_GET_CONFIG, U_SET_CONFIG, U_RTH_REQUEST, U_ARM_REQUEST,
  U_DISARM_REQUEST, U_PING, U_GET_DIAGNOSTICS, U_GET_PROTOCOL_INFO, U_SET_RTH_ALT, U_GET_RTH_ALT, U_COUNT
};
static const char *const UCMD[U_COUNT] = {
  "", "HELLO", "GET_STATUS", "GET_REMOTE_STATUS", "GET_FC_STATUS", "GET_TELEMETRY", "GET_BATTERY", "GET_GPS",
  "GET_FLIGHT_DATA", "GET_SENSOR_STATUS", "GET_LINK_STATS", "GET_HOME_POINT", "GET_FLIGHT_MODE", "GET_PID", "SET_PID",
  "CALIBRATE_REQUEST", "GET_FLIGHT_LOGS", "CLEAR_FLIGHT_LOGS", "GET_CONFIG", "SET_CONFIG", "RTH_REQUEST", "ARM_REQUEST",
  "DISARM_REQUEST", "PING", "GET_DIAGNOSTICS", "GET_PROTOCOL_INFO", "SET_RTH_ALT", "GET_RTH_ALT"
};
#define RTH_ALT_MIN_M 10   // [RA] harus sama dgn lim::RTH_MAX_ALT_MIN di FC dan RTH_ALT_MIN_M di APK
#define RTH_ALT_MAX_M 120  // [RA]

struct AxisCal { uint16_t mn, ct, mx; };
struct __attribute__((packed)) CalBlock { uint32_t magic; uint16_t ver; uint8_t calibrated; uint8_t pad; AxisCal ax[4]; uint16_t crc; };

struct __attribute__((packed)) Config {
  uint8_t channel, dataRate, paLevel; uint8_t addr[5];
  uint16_t deadzone;      // permil
  uint8_t invert;         // bit0..3 = LX,LY,RX,RY
  uint8_t filterAlpha;    // % bobot sampel baru
  uint16_t gestureMs;
  uint8_t controlHz, stickMode, apkArm;
  uint16_t linkTimeoutMs, reqTimeoutMs, telemMaxAgeMs, gestureThr;
};
struct __attribute__((packed)) CfgBlock { uint32_t magic; uint16_t ver; Config c; uint16_t crc; };

struct __attribute__((packed)) TBasic { uint8_t fcState, flightMode, preflight, failsafe, rth, homeSet; uint16_t flightTimeS; uint16_t errFlags; uint8_t armReject, fwMaj, fwMin; };
struct __attribute__((packed)) TBatt { uint16_t mV; int16_t mA; uint16_t mAh; uint8_t pct, status; };
struct __attribute__((packed)) TGps { uint8_t fix, sats; int32_t lat, lon; uint16_t hdop; int16_t altDm; uint16_t speedCms; };
struct __attribute__((packed)) TSens { uint16_t present, healthy, calibrated, errors, dataAgeMs; };
struct __attribute__((packed)) TFlight { int16_t altDm; uint8_t altSrc; uint16_t speedCms, homeDistM; int16_t headingDeg, rollDd, pitchDd, yawDd; uint16_t thrCmd; };
struct __attribute__((packed)) TFcLink { uint16_t rxOk, rxBad, ctrlGap; uint8_t linkState, failsafe; };
struct TCache { bool valid, unsupported; uint32_t rxMs; };

struct Slot {
  bool used, delivered;
  uint8_t cmd, src, ptype, expect, reqId, plen, tries, maxTries;
  uint8_t pl[PKT_MAX_PL];
  uint32_t qMs, lastTx, timeoutMs, wait[3];
  uint8_t nWait, auxLen, aux[16];
};

// ======================= GLOBAL =======================
static Config cfg;
static bool cfgDefaulted = false, calLoaded = false;
static AxisCal cal[4], calWork[4];
static bool calibrated = false;
static CalState calState = CAL_NONE;
static uint32_t calStartMs = 0, calSum[4], calN = 0;
static bool joySampled = false;

static uint16_t rawAdc[4];
static float filt[4];
static bool filtInit = false;
static int16_t axisVal[4];
static bool adcOk[4], joyValid = false, allNeutral = false;
static uint8_t badCnt[4];
static uint32_t neutralSince = 0, lastJoyMs = 0, notNeutralSince = 0;
static bool armLatchClear = false;

static RemoteState rstate = RS_CALIBRATION_REQUIRED;
static uint32_t stateSinceMs = 0, gStart = 0, pendStart = 0;
static const char *lastArmBlock = "NONE";
static bool rthPressed = false;

static RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
static bool radioReady = false, txInFlight = false, everConnected = false, rxHaveSeq = false, lastRpd = false;
static uint8_t txSeq = 0, lastRxSeq = 0, reqCounter = 0;
static int8_t txSlot = -1;
static uint32_t txStartMs = 0, lastTickMs = 0, lastHelloMs = 0, lastValidRxMs = 0, lastAckMs = 0, lastRadioInitTry = 0;
static LinkState linkState = LS_NOT_INIT;
static uint32_t stTx, stTxOk, stTxFail, stTxTimeout, stRetries, stRxValid, stRxBad, stRxDup, stRxLost, stRxUnknown, stRxLenBad, stReqTimeout;
static uint32_t winTx = 0, winFail = 0, lastLossPct = 0;
static uint8_t wantIdx = 0;

static TCache tc[T_COUNT];
static TBasic tb; static TBatt tbat; static TGps tgps; static TSens tsens; static TFlight tfl; static TFcLink tlnk;
static bool capsKnown = false, basicFresh = false, fcKnown = false;
static uint32_t fcCaps = 0; static uint8_t fcFwMaj = 0, fcFwMin = 0, fcId = 0, pairStatus = 255, pairReason = 0;
static uint8_t rthState = 0, rthReason = 0;
static Slot slots[NSLOT];

static char lineBuf[LINE_MAX + 1];
static uint16_t lineLen = 0; static bool lineOver = false; static uint32_t lineStartMs = 0, lastUsbRxMs = 0;
static uint32_t usbDrops = 0, usbParseErr = 0, usbOverflow = 0, usbTrunc = 0; static bool usbDirty = false;
static char jb[1400]; static int jlen = 0; static bool jtrunc = false;

static uint32_t loopMaxUs = 0, loopSumUs = 0, loopCnt = 0, loopMaxUsPrev = 0, loopAvgUsPrev = 0, diagWinMs = 0, lastLoopUs = 0;
static uint16_t errFlagsRemote = 0;

static void setState(RemoteState ns, const char *reason);
static void setStateCalEnd(const char *reason);
static void emitEvent(const char *name, const char *fmt, ...);
static void finishSlot(int i, const char *err, const uint8_t *pl, uint8_t plen);
static void onLinkLost();

// ======================= UTIL =======================
static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) { c ^= (uint16_t)(*d++) << 8; for (uint8_t i = 0; i < 8; i++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1); }
  return c;
}
#define BIN_FLAG 0x40000000UL   // bit id: permintaan berasal dari protokol biner
static bool usbBinary = false;
struct Slot;
static void binError(uint32_t id, const char *code);
static bool usbActive() { return lastUsbRxMs && (millis() - lastUsbRxMs) < USB_ACTIVE_MS; }

static const char *rsName(RemoteState s) {
  switch (s) {
    case RS_CALIBRATION_REQUIRED: return "CALIBRATION_REQUIRED"; case RS_CALIBRATING: return "CALIBRATING";
    case RS_DISARMED: return "DISARMED"; case RS_ARM_GESTURE: return "ARM_GESTURE"; case RS_ARM_PENDING: return "ARM_PENDING";
    case RS_ARMED_IDLE: return "ARMED_IDLE"; case RS_FLIGHT: return "FLIGHT"; }
  return "UNKNOWN";
}
static const char *fcName(uint8_t s) {
  switch (s) { case FC_DISARMED: return "DISARMED"; case FC_ARMING: return "ARMING"; case FC_ARMED: return "ARMED"; case FC_FAILSAFE: return "FAILSAFE"; case FC_ERROR: return "ERROR"; }
  return "UNKNOWN";
}
static const char *linkName(LinkState s) {
  switch (s) { case LS_NOT_INIT: return "NOT_INITIALIZED"; case LS_SEARCHING: return "SEARCHING"; case LS_CONNECTED: return "CONNECTED"; case LS_LOST: return "LINK_LOST"; }
  return "UNKNOWN";
}
static const char *rejName(uint8_t r) {
  switch (r) { case 0: return "NONE"; case 1: return "IMU_NOT_READY"; case 2: return "CALIBRATION"; case 3: return "BATTERY_LOW"; case 4: return "NO_LINK";
    case 5: return "THROTTLE_NOT_LOW"; case 6: return "SENSOR_FAULT"; case 7: return "FAILSAFE_ACTIVE"; case 8: return "OTHER"; }
  return "UNDEFINED";
}
static const char *fcErrName(uint8_t c) {
  switch (c) { case 1: return "UNSUPPORTED"; case 2: return "REJECTED"; case 3: return "BUSY"; case 4: return "BAD_PAYLOAD"; case 5: return "NOT_READY"; }
  return "FC_ERROR";
}
static const char *reqName(uint8_t c) {
  switch (c) { case C_ARM: return "ARM_REQUEST"; case C_DISARM: return "DISARM_REQUEST"; case C_RTH: return "RTH_REQUEST"; case C_GET_PID: return "GET_PID";
    case C_SET_PID: return "SET_PID"; case C_CALIB: return "CALIBRATE_REQUEST"; case C_LOG: return "GET_FLIGHT_LOGS"; case C_CLEAR_LOG: return "CLEAR_FLIGHT_LOGS";
    case C_HOME: return "GET_HOME_POINT"; case C_RTHALT: return "SET_RTH_ALT"; }
  return "UNKNOWN";
}
static const char *axisName(uint8_t a) { static const char *n[4] = {"roll", "pitch", "yaw", "altitude"}; return a < 4 ? n[a] : "?"; }
static const char *B(bool b) { return b ? "true" : "false"; }

// ======================= USB OUTPUT =======================
static void usbWrite(const char *b, int n) {
  uint32_t t0 = millis(); int off = 0;
  if (usbDirty) { if (usbBinary) usbDirty = false; else if (Serial.availableForWrite() > 0) { Serial.write((uint8_t)'\n'); usbDirty = false; } }
  while (off < n) {
    int av = Serial.availableForWrite();
    if (av > 0) { int k = av < (n - off) ? av : (n - off); Serial.write((const uint8_t *)b + off, k); off += k; }
    else { if (millis() - t0 >= 6) { usbDrops++; usbDirty = true; return; } yield(); }
  }
}
static void jadd(const char *fmt, ...) {
  va_list ap; va_start(ap, fmt);
  int r = vsnprintf(jb + jlen, sizeof(jb) - jlen, fmt, ap);
  va_end(ap);
  if (r < 0 || jlen + r >= (int)sizeof(jb) - 2) { jtrunc = true; jlen = sizeof(jb) - 2; jb[jlen] = 0; } else jlen += r;
}
static void jbeg(const char *type, bool hasId, uint32_t id) {
  jlen = 0; jtrunc = false; jb[0] = 0;
  jadd("{\"type\":\"%s\"", type);
  if (hasId) jadd(",\"id\":%lu", (unsigned long)id);
  jadd(",\"version\":1");
}
static void sendError(bool hasId, uint32_t id, const char *code, const char *req, const char *msg) {
  if (hasId && (id & BIN_FLAG)) { binError(id, code); return; }
  if (usbBinary) return;
  char e[220];
  int n = snprintf(e, sizeof(e), "{\"type\":\"ERROR\"%s", "");
  if (hasId) n += snprintf(e + n, sizeof(e) - n, ",\"id\":%lu", (unsigned long)id);
  n += snprintf(e + n, sizeof(e) - n, ",\"version\":1,\"code\":\"%s\",\"request\":\"%s\",\"message\":\"%s\"}\n", code, req, msg);
  if (n > (int)sizeof(e) - 1) n = sizeof(e) - 1;
  usbWrite(e, n);
}
static void emitEvent(const char *name, const char *fmt, ...) {
  if (!usbActive() || usbBinary) return;
  char e[240];
  int n = snprintf(e, sizeof(e), "{\"type\":\"EVENT\",\"version\":1,\"event\":\"%s\"", name);
  if (fmt && fmt[0]) { va_list ap; va_start(ap, fmt); n += vsnprintf(e + n, sizeof(e) - n, fmt, ap); va_end(ap); }
  if (n > (int)sizeof(e) - 3) n = sizeof(e) - 3;
  e[n++] = '}'; e[n++] = '\n';
  usbWrite(e, n);
}
static void sendUsbMessage() { // akhiri dan kirim pesan di jb
  if (jtrunc) { sendError(false, 0, "OUTPUT_TOO_LARGE", "-", "respons terlalu besar"); return; }
  jadd("}"); if (jtrunc) { sendError(false, 0, "OUTPUT_TOO_LARGE", "-", "respons terlalu besar"); return; }
  jb[jlen++] = '\n'; usbWrite(jb, jlen);
}
static void jN(const char *k, long v, bool na) { if (na) jadd(",\"%s\":null", k); else jadd(",\"%s\":%ld", k, v); }

// ======================= JSON MINI PARSER =======================
static bool jsonBasicValid(const char *s) {
  size_t n = strlen(s), i = 0; int d = 0;
  while (i < n && s[i] == ' ') i++;
  if (i >= n || s[i] != '{') return false;
  for (; i < n; i++) {
    char c = s[i];
    if (c == '"') { i++; while (i < n && s[i] != '"') { if (s[i] == '\\') i++; i++; } if (i >= n) return false; }
    else if (c == '{' || c == '[') d++;
    else if (c == '}' || c == ']') { d--; if (d < 0) return false; if (d == 0) { for (i++; i < n; i++) if (s[i] != ' ') return false; return true; } }
  }
  return false;
}
static bool jFind(const char *s, const char *parent, const char *key, const char **v, int *vl, bool *isStr) {
  int depth = 0; char pkey[4][20]; memset(pkey, 0, sizeof(pkey));
  const int want = parent ? 2 : 1; size_t n = strlen(s), i = 0;
  while (i < n) {
    char c = s[i];
    if (c == '{') { depth++; i++; continue; }
    if (c == '}') { depth--; i++; continue; }
    if (c != '"') { i++; continue; }
    size_t a = ++i;
    while (i < n && s[i] != '"') { if (s[i] == '\\') i++; i++; }
    if (i >= n) return false;
    size_t b = i; i++;
    size_t j = i; while (j < n && s[j] == ' ') j++;
    if (j >= n || s[j] != ':') { i = j; continue; }
    char k[20]; size_t kl = b - a; if (kl >= sizeof(k)) kl = sizeof(k) - 1; memcpy(k, s + a, kl); k[kl] = 0;
    j++; while (j < n && s[j] == ' ') j++;
    if (j >= n) return false;
    if (s[j] == '{') { if (depth + 1 < 4) { strncpy(pkey[depth + 1], k, 19); } i = j; continue; }
    const char *vs; int len; bool str = false; size_t endi;
    if (s[j] == '"') {
      size_t k2 = j + 1; while (k2 < n && s[k2] != '"') { if (s[k2] == '\\') k2++; k2++; }
      if (k2 >= n) return false;
      vs = s + j + 1; len = (int)(k2 - j - 1); str = true; endi = k2 + 1;
    } else if (s[j] == '[') {
      int d = 0; size_t k2 = j;
      while (k2 < n) {
        if (s[k2] == '"') { k2++; while (k2 < n && s[k2] != '"') { if (s[k2] == '\\') k2++; k2++; } }
        else if (s[k2] == '[') d++;
        else if (s[k2] == ']') { d--; if (d == 0) { k2++; break; } }
        k2++;
      }
      i = k2; continue;
    } else {
      size_t k2 = j; while (k2 < n && s[k2] != ',' && s[k2] != '}' && s[k2] != ' ') k2++;
      vs = s + j; len = (int)(k2 - j); endi = k2;
    }
    if (depth == want && strcmp(k, key) == 0 && (!parent || strcmp(pkey[depth], parent) == 0)) { *v = vs; *vl = len; *isStr = str; return true; }
    i = endi;
  }
  return false;
}
static int jNum(const char *s, const char *parent, const char *key, long *out) { // 0 tidak ada, 1 ok, -1 invalid
  const char *v; int vl; bool st;
  if (!jFind(s, parent, key, &v, &vl, &st)) return 0;
  if (st || vl <= 0 || vl > 11) return -1;
  char t[12]; memcpy(t, v, vl); t[vl] = 0; char *e; long r = strtol(t, &e, 10);
  if (*e) return -1; *out = r; return 1;
}
static int jFlt(const char *s, const char *parent, const char *key, float *out) {
  const char *v; int vl; bool st;
  if (!jFind(s, parent, key, &v, &vl, &st)) return 0;
  if (st || vl <= 0 || vl > 15) return -1;
  char t[16]; memcpy(t, v, vl); t[vl] = 0; char *e; double r = strtod(t, &e);
  if (*e || !isfinite(r)) return -1; *out = (float)r; return 1;
}
static int jStrV(const char *s, const char *parent, const char *key, char *dst, size_t cap) {
  const char *v; int vl; bool st;
  if (!jFind(s, parent, key, &v, &vl, &st)) return 0;
  if (!st || vl < 0 || (size_t)vl >= cap) return -1;
  memcpy(dst, v, vl); dst[vl] = 0; return 1;
}
static int jBool(const char *s, const char *parent, const char *key, bool *out) {
  const char *v; int vl; bool st;
  if (!jFind(s, parent, key, &v, &vl, &st)) return 0;
  if (st) return -1;
  if (vl == 4 && !strncmp(v, "true", 4)) { *out = true; return 1; }
  if (vl == 5 && !strncmp(v, "false", 5)) { *out = false; return 1; }
  return -1;
}

// ======================= KONFIGURASI & KALIBRASI (EEPROM) =======================
static void defaultConfig(Config &c) {
  memset(&c, 0, sizeof(c));
  c.channel = 76; c.dataRate = 1; c.paLevel = 2;
  c.addr[0] = 'N'; c.addr[1] = 'X'; c.addr[2] = 'F'; c.addr[3] = 'C'; c.addr[4] = '1';
  c.deadzone = 60; c.invert = 0; c.filterAlpha = 60; c.gestureMs = 1500; c.controlHz = 50; c.stickMode = 2; c.apkArm = 0;
  c.linkTimeoutMs = 500; c.reqTimeoutMs = 800; c.telemMaxAgeMs = 1000; c.gestureThr = 800;
}
static bool cfgValid(const Config &c) {
  bool same = true; for (int i = 1; i < 5; i++) if (c.addr[i] != c.addr[0]) same = false;
  return c.channel <= 125 && c.dataRate <= 2 && c.paLevel <= 3 && !same &&
         c.deadzone >= 10 && c.deadzone <= 300 && (c.invert & 0xF0) == 0 && c.filterAlpha >= 5 && c.filterAlpha <= 100 &&
         c.gestureMs >= 500 && c.gestureMs <= 5000 && c.controlHz >= 20 && c.controlHz <= 100 && (c.stickMode == 1 || c.stickMode == 2) &&
         c.apkArm <= 1 && c.linkTimeoutMs >= 200 && c.linkTimeoutMs <= 5000 && c.reqTimeoutMs >= 200 && c.reqTimeoutMs <= 5000 &&
         c.telemMaxAgeMs >= 300 && c.telemMaxAgeMs <= 10000 && c.gestureThr >= 500 && c.gestureThr <= 950;
}
static bool saveConfig(const Config &c) {
  CfgBlock b; memset(&b, 0, sizeof(b)); b.magic = EE_MAGIC_CFG; b.ver = 1; b.c = c;
  b.crc = crc16((const uint8_t *)&b, offsetof(CfgBlock, crc));
  EEPROM.put(EE_CFG_ADDR, b); if (!EEPROM.commit()) return false;
  CfgBlock r; EEPROM.get(EE_CFG_ADDR, r); return memcmp(&b, &r, sizeof(b)) == 0;
}
static void loadConfig() {
  CfgBlock b; EEPROM.get(EE_CFG_ADDR, b);
  if (b.magic == EE_MAGIC_CFG && b.ver == 1 && b.crc == crc16((const uint8_t *)&b, offsetof(CfgBlock, crc)) && cfgValid(b.c)) { cfg = b.c; cfgDefaulted = false; }
  else { defaultConfig(cfg); cfgDefaulted = true; }
}
static bool calValid(const AxisCal *c) {
  for (int i = 0; i < 4; i++) {
    if (!(c[i].mn < c[i].ct && c[i].ct < c[i].mx)) return false;
    if (c[i].ct - c[i].mn < CAL_MIN_SPAN || c[i].mx - c[i].ct < CAL_MIN_SPAN) return false;
    if (c[i].ct < 1200 || c[i].ct > 2900 || c[i].mx > 4095) return false;
  }
  return true;
}
static bool saveCalibration(const AxisCal *c, bool ok) {
  CalBlock b; memset(&b, 0, sizeof(b)); b.magic = EE_MAGIC_CAL; b.ver = 1; b.calibrated = ok ? 1 : 0;
  if (ok) memcpy(b.ax, c, sizeof(b.ax));
  b.crc = crc16((const uint8_t *)&b, offsetof(CalBlock, crc));
  EEPROM.put(EE_CAL_ADDR, b); if (!EEPROM.commit()) return false;
  CalBlock r; EEPROM.get(EE_CAL_ADDR, r); return memcmp(&b, &r, sizeof(b)) == 0;
}
static void loadCalibration() {
  CalBlock b; EEPROM.get(EE_CAL_ADDR, b);
  calibrated = false; calLoaded = false;
  if (b.magic == EE_MAGIC_CAL && b.ver == 1 && b.crc == crc16((const uint8_t *)&b, offsetof(CalBlock, crc)) && b.calibrated == 1 && calValid(b.ax)) {
    memcpy(cal, b.ax, sizeof(cal)); calibrated = true; calLoaded = true;
  }
}

// ======================= JOYSTICK =======================
static void initJoystick() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // rentang ~0..3.1V
  for (int i = 0; i < 4; i++) { pinMode(PIN_JOY[i], INPUT); axisVal[i] = 0; adcOk[i] = false; badCnt[i] = 0; }
}
static int16_t normalizeAxis(int i, float f) {
  const AxisCal &c = cal[i]; float v;
  if (f >= c.ct) v = (f - c.ct) / (float)(c.mx - c.ct); else v = (f - c.ct) / (float)(c.ct - c.mn);
  if (v > 1.0f) v = 1.0f; if (v < -1.0f) v = -1.0f;
  float dz = cfg.deadzone / 1000.0f, a = fabsf(v);
  if (a < dz) return 0;
  a = (a - dz) / (1.0f - dz); v = (v < 0) ? -a : a;
  if (cfg.invert & (1 << i)) v = -v;
  long r = lroundf(v * 1000.0f); if (r > 1000) r = 1000; if (r < -1000) r = -1000;
  return (int16_t)r;
}
static void readJoystick() {
  uint32_t now = millis(); joySampled = false;
  if (now - lastJoyMs < JOY_PERIOD_MS) return;
  lastJoyMs = now; joySampled = true;
  float a = cfg.filterAlpha / 100.0f;
  for (int i = 0; i < 4; i++) {
    uint32_t s = analogRead(PIN_JOY[i]); s += analogRead(PIN_JOY[i]);
    uint16_t r = (uint16_t)(s / 2); rawAdc[i] = r;
    if (r < RAW_MIN_OK || r > RAW_MAX_OK) { if (badCnt[i] < 255) badCnt[i]++; } else badCnt[i] = 0;
    if (!filtInit) filt[i] = r; else filt[i] += a * ((float)r - filt[i]);
  }
  filtInit = true; joyValid = true;
  for (int i = 0; i < 4; i++) {
    bool ok = badCnt[i] < 5;
    if (ok && calibrated && calState == CAL_NONE && (filt[i] < (float)cal[i].mn - 400.0f || filt[i] > (float)cal[i].mx + 400.0f)) ok = false;
    adcOk[i] = ok; if (!ok) joyValid = false;
  }
  bool use = joyValid && calibrated && calState == CAL_NONE; allNeutral = use;
  for (int i = 0; i < 4; i++) { axisVal[i] = use ? normalizeAxis(i, filt[i]) : 0; if (axisVal[i] != 0) allNeutral = false; }
  if (allNeutral) { if (!neutralSince) neutralSince = now; } else neutralSince = 0;
}
static void calibrateJoystick() {
  if (calState == CAL_NONE || !joySampled) return;
  uint32_t now = millis();
  if (calState == CAL_CAPTURE) {
    for (int i = 0; i < 4; i++) calSum[i] += rawAdc[i];
    calN++;
    if (now - calStartMs >= 700 && calN > 20) {
      bool ok = true;
      for (int i = 0; i < 4; i++) { uint16_t ct = (uint16_t)(calSum[i] / calN); calWork[i].mn = calWork[i].ct = calWork[i].mx = ct; if (ct < 1200 || ct > 2900) ok = false; }
      if (ok) { calState = CAL_SWEEP; calStartMs = now; emitEvent("CALIBRATION", ",\"state\":\"SWEEP\",\"hint\":\"gerakkan semua stik ke seluruh batas lalu kirim finish\""); }
      else { calState = CAL_NONE; setStateCalEnd("CENTER_OUT_OF_RANGE"); }
    }
  } else if (calState == CAL_SWEEP) {
    for (int i = 0; i < 4; i++) { uint16_t v = (uint16_t)filt[i]; if (v < calWork[i].mn) calWork[i].mn = v; if (v > calWork[i].mx) calWork[i].mx = v; }
    if (now - calStartMs > CAL_TIMEOUT_MS) { calState = CAL_NONE; setStateCalEnd("CAL_TIMEOUT"); }
  }
}

// ======================= STATE =======================
static void setState(RemoteState ns, const char *reason) {
  if (ns == rstate) return;
  rstate = ns; stateSinceMs = millis();
  emitEvent("ARM_STATE", ",\"state\":\"%s\",\"reason\":\"%s\"", rsName(ns), reason);
}
static void setStateCalEnd(const char *reason) {
  setState(calibrated ? RS_DISARMED : RS_CALIBRATION_REQUIRED, reason);
  emitEvent("CALIBRATION", ",\"state\":\"ENDED\",\"reason\":\"%s\"", reason);
}
static bool armedLike() { return rstate == RS_ARM_PENDING || rstate == RS_ARMED_IDLE || rstate == RS_FLIGHT; }
static bool gestureNow() {
  if (!joyValid || !calibrated || calState != CAL_NONE) return false;
  int t = cfg.gestureThr;
  return axisVal[0] >= t && axisVal[1] <= -t && axisVal[2] <= -t && axisVal[3] <= -t;
}
static const char *armBlockReason() {
  if (!calibrated) return "CALIBRATION_REQUIRED";
  if (calState != CAL_NONE) return "CALIBRATING";
  if (!joyValid) return "JOYSTICK_INVALID";
  if (!radioReady) return "RADIO_NOT_INIT";
  if (linkState != LS_CONNECTED) return "NO_LINK";
  if (!capsKnown || !fcKnown) return "FC_NOT_READY";
  if (!(fcCaps & CAP_ARM)) return "ARM_UNSUPPORTED";
  if (tb.fcState != FC_DISARMED) return "FC_NOT_DISARMED";
  if (tb.failsafe) return "FC_FAILSAFE";
  if (tb.preflight != 1) return "PREFLIGHT_NOT_OK";
  return NULL;
}

// ======================= RADIO =======================
static bool initRadio() {
  radioReady = false; txInFlight = false;
  SPI.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, PIN_NRF_CSN);
  if (!radio.begin(&SPI)) return false;
  static const rf24_pa_dbm_e PA[4] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
  static const rf24_datarate_e DR[3] = {RF24_250KBPS, RF24_1MBPS, RF24_2MBPS};
  radio.setPALevel(PA[cfg.paLevel]);
  radio.setDataRate(DR[cfg.dataRate]);
  radio.setChannel(cfg.channel);
  radio.setAddressWidth(5);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  uint8_t ard = (cfg.dataRate == 0) ? 7 : 4;   // 2000us (250k) / 1250us (1M,2M)
  radio.setRetries(ard, 4);
  radio.openWritingPipe(cfg.addr);
  radio.stopListening();
  radio.flush_tx(); radio.flush_rx();
  radioReady = true; return true;
}
static bool sendRadioPacket(uint8_t type, uint8_t reqId, const uint8_t *pl, uint8_t plen) {
  if (!radioReady || txInFlight || plen > PKT_MAX_PL) return false;
  uint8_t b[32];
  b[0] = PKT_MAGIC; b[1] = PROTO_VER; b[2] = type; b[3] = ++txSeq; b[4] = reqId; b[5] = plen;
  if (plen) memcpy(b + PKT_HDR, pl, plen);
  uint16_t c = crc16(b, PKT_HDR + plen); b[PKT_HDR + plen] = c & 0xFF; b[PKT_HDR + plen + 1] = c >> 8;
  radio.startWrite(b, PKT_HDR + plen + 2, false);
  txInFlight = true; txStartMs = millis(); stTx++;
  return true;
}
static bool validatePacket(const uint8_t *b, uint8_t n, uint8_t *type, uint8_t *seq, uint8_t *req, const uint8_t **pl, uint8_t *plen) {
  if (n < PKT_HDR + 2 || n > 32) { stRxLenBad++; return false; }
  if (b[0] != PKT_MAGIC || b[1] != PROTO_VER) { stRxUnknown++; return false; }
  uint8_t l = b[5];
  if (l > PKT_MAX_PL || (uint8_t)(PKT_HDR + l + 2) != n) { stRxLenBad++; return false; }
  uint16_t c = crc16(b, PKT_HDR + l), r = b[PKT_HDR + l] | ((uint16_t)b[PKT_HDR + l + 1] << 8);
  if (c != r) { stRxBad++; return false; }
  *type = b[2]; *seq = b[3]; *req = b[4]; *pl = b + PKT_HDR; *plen = l; return true;
}
static int tIdx(uint8_t type) {
  switch (type) { case PT_TELEM_BASIC: return T_BASIC; case PT_TELEM_BATTERY: return T_BATT; case PT_TELEM_GPS: return T_GPS;
    case PT_TELEM_SENSORS: return T_SENS; case PT_TELEM_FLIGHT: return T_FLIGHT; case PT_TELEM_LINK: return T_LINK; }
  return -1;
}
static void invalidateTelemetry() { for (int i = 0; i < T_COUNT; i++) tc[i].valid = false; }

// ---- Slot permintaan ----
static uint8_t nextReqId() { if (++reqCounter == 0) reqCounter = 1; return reqCounter; }
static int enqueueReq(uint8_t cmd, uint8_t src, uint8_t ptype, uint8_t expect, const uint8_t *pl, uint8_t plen, uint8_t maxTries,
                      uint32_t timeoutMs, bool hasId, uint32_t id, bool coalesce, const uint8_t *aux, uint8_t auxLen) {
  if (coalesce) {
    for (int i = 0; i < NSLOT; i++) {
      Slot &s = slots[i];
      if (s.used && s.cmd == cmd && s.plen == plen && (plen == 0 || memcmp(s.pl, pl, plen) == 0)) {
        if (hasId) { if (s.nWait >= 3) return -2; s.wait[s.nWait++] = id; }
        return i;
      }
    }
  }
  for (int i = 0; i < NSLOT; i++) {
    Slot &s = slots[i];
    if (s.used) continue;
    memset(&s, 0, sizeof(s));
    s.used = true; s.cmd = cmd; s.src = src; s.ptype = ptype; s.expect = expect; s.reqId = nextReqId();
    s.plen = plen; if (plen) memcpy(s.pl, pl, plen);
    s.maxTries = maxTries; s.timeoutMs = timeoutMs; s.qMs = millis();
    if (hasId) s.wait[s.nWait++] = id;
    if (aux && auxLen <= sizeof(s.aux)) { memcpy(s.aux, aux, auxLen); s.auxLen = auxLen; }
    return i;
  }
  return -1;
}
static bool slotBusy(uint8_t cmd) { for (int i = 0; i < NSLOT; i++) if (slots[i].used && slots[i].cmd == cmd) return true; return false; }

static bool respondSlot(Slot &s, uint32_t id, const uint8_t *pl, uint8_t plen) {
  switch (s.cmd) {
    case C_ARM: {
      if (plen < 2) return false;
      jbeg("ARM", true, id);
      jadd(",\"accepted\":%s,\"fc_arm_state\":\"%s\",\"reject_code\":%u,\"reject\":\"%s\"", B((pl[0] == FC_ARMED || pl[0] == FC_ARMING) && pl[1] == 0), fcName(pl[0]), pl[1], rejName(pl[1]));
      sendUsbMessage(); return true;
    }
    case C_DISARM: case C_HOME: {
      if (plen < 3) return false;
      if (s.cmd == C_DISARM) {
        jbeg("DISARM", true, id);
        jadd(",\"fc_status\":%u,\"fc_reason\":%u,\"note\":\"status DISARM final hanya dari telemetry FC\"", pl[1], pl[2]);
      } else {
        jbeg("HOME_POINT", true, id);
        jadd(",\"fc_status\":%u", pl[1]);
        if (pl[1] == 0 && plen >= 11) {
          int32_t la, lo; memcpy(&la, pl + 3, 4); memcpy(&lo, pl + 7, 4);
          jadd(",\"home_set\":%s,\"lat\":%.7f,\"lon\":%.7f", B(pl[2] != 0), (double)la * 1e-7, (double)lo * 1e-7);
        }
      }
      sendUsbMessage(); return true;
    }
    case C_RTHALT: {   // [RA] balasan FC: [3][status][0][alt lo][alt hi]; alt = nilai SEBENARNYA yang tersimpan di FC
      if (plen < 5) return false;
      uint16_t alt = (uint16_t)pl[3] | ((uint16_t)pl[4] << 8);
      bool isSet = (s.plen >= 3); uint16_t rq = isSet ? ((uint16_t)s.pl[1] | ((uint16_t)s.pl[2] << 8)) : 0;
      jbeg(isSet ? "RTH_ALT_RESULT" : "RTH_ALT", true, id);
      jadd(",\"fc_status\":%u,\"applied\":%s,\"alt_m\":%u", pl[1], B(pl[1] == 0), alt);
      if (isSet) jadd(",\"requested_m\":%u,\"readback_matches\":%s", rq, B(pl[1] == 0 && alt == rq));
      sendUsbMessage(); return true;
    }
    case C_RTH: {
      if (plen < 2) return false;
      jbeg("RTH", true, id);
      jadd(",\"rth_state\":%u,\"reason_code\":%u,\"accepted\":%s", pl[0], pl[1], B(pl[0] == 2));
      sendUsbMessage(); return true;
    }
    case C_GET_PID: {
      if (plen < 16) return false;
      float kp, ki, kd; uint16_t cc; memcpy(&kp, pl + 2, 4); memcpy(&ki, pl + 6, 4); memcpy(&kd, pl + 10, 4); memcpy(&cc, pl + 14, 2);
      jbeg("PID", true, id);
      jadd(",\"axis\":\"%s\",\"fc_status\":%u", axisName(pl[0]), pl[1]);
      if (pl[1] == 0) jadd(",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"cfg_crc\":%u", kp, ki, kd, cc);
      sendUsbMessage(); return true;
    }
    case C_SET_PID: {
      if (plen < 17 || s.auxLen < 13) return false;
      float kp, ki, kd, rq[3]; uint16_t cc; memcpy(&kp, pl + 3, 4); memcpy(&ki, pl + 7, 4); memcpy(&kd, pl + 11, 4); memcpy(&cc, pl + 15, 2);
      memcpy(rq, s.aux + 1, 12);
      bool match = fabsf(kp - rq[0]) < 0.001f && fabsf(ki - rq[1]) < 0.001f && fabsf(kd - rq[2]) < 0.001f;
      jbeg("PID_RESULT", true, id);
      jadd(",\"axis\":\"%s\",\"result\":\"%s\",\"persisted\":%s,\"readback_matches\":%s,\"reason_code\":%u,\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"cfg_crc\":%u",
           axisName(pl[0]), pl[1] == 2 ? "PERSISTED" : (pl[1] == 1 ? "APPLIED_RAM_ONLY" : "REJECTED"), B(pl[1] == 2), B(match), pl[2], kp, ki, kd, cc);
      sendUsbMessage(); return true;
    }
    case C_CALIB: {
      if (plen < 4) return false;
      static const char *st[6] = {"IDLE", "RUNNING", "DONE", "FAILED", "UNSUPPORTED", "REJECTED"};
      jbeg("CALIBRATION", true, id);
      jadd(",\"target_code\":%u,\"state\":\"%s\",\"progress\":%u,\"reason_code\":%u", pl[0], pl[1] < 6 ? st[pl[1]] : "UNKNOWN", pl[2], pl[3]);
      sendUsbMessage(); return true;
    }
    case C_LOG: case C_CLEAR_LOG: {
      if (plen < 6) return false;
      static const char *st[5] = {"OK", "NO_LOGGING", "BAD_INDEX", "BUSY", "REJECTED"};
      uint16_t idx, tot; memcpy(&idx, pl + 2, 2); memcpy(&tot, pl + 4, 2);
      jbeg(s.cmd == C_LOG ? "FLIGHT_LOGS" : "LOGS_CLEARED", true, id);
      jadd(",\"op\":%u,\"status\":\"%s\",\"index\":%u,\"total\":%u", pl[0], pl[1] < 5 ? st[pl[1]] : "UNKNOWN", idx, tot);
      if (s.cmd == C_LOG && pl[1] == 0 && plen > 6) { jadd(",\"data_hex\":\""); for (int i = 6; i < plen; i++) jadd("%02X", pl[i]); jadd("\""); }
      sendUsbMessage(); return true;
    }
  }
  return false;
}
static bool binRespond(Slot &s, uint32_t id, const uint8_t *pl, uint8_t plen);
static void finishSlot(int i, const char *err, const uint8_t *pl, uint8_t plen) {
  Slot &s = slots[i];
  if (s.cmd == C_ARM && err && rstate == RS_ARM_PENDING) { armLatchClear = false; setState(RS_DISARMED, err); }
  if (s.src == SRC_BUTTON || s.src == SRC_GESTURE) {
    if (s.cmd == C_RTH) {
      if (err) emitEvent("RTH_RESULT", ",\"accepted\":false,\"reason\":\"%s\"", err);
      else if (plen >= 2) emitEvent("RTH_RESULT", ",\"accepted\":%s,\"rth_state\":%u,\"reason_code\":%u", B(pl[0] == 2), pl[0], pl[1]);
    }
    s.used = false; return;
  }
  for (uint8_t w = 0; w < s.nWait; w++) {
    uint32_t id = s.wait[w];
    if (id & BIN_FLAG) { if (err) binError(id, err); else if (!binRespond(s, id, pl, plen)) binError(id, "BAD_RESPONSE"); continue; }
    if (err) {
      char d[40] = ""; if (pl && plen >= 3 && !strcmp(err, fcErrName(pl[1]))) snprintf(d, sizeof(d), "fc_reason=%u", pl[2]);
      sendError(true, id, err, reqName(s.cmd), d);
    } else if (!respondSlot(s, id, pl, plen)) sendError(true, id, "BAD_RESPONSE", reqName(s.cmd), "payload FC tidak valid");
  }
  s.used = false;
}
static void failAllSlots(const char *why) { for (int i = 0; i < NSLOT; i++) if (slots[i].used) finishSlot(i, why, NULL, 0); }

// ---- Penanganan paket masuk dari FC ----
static void onArmStatus(const uint8_t *pl, uint8_t plen) {
  if (plen < 2) return;
  if (rstate == RS_ARM_PENDING) {
    if (pl[0] == FC_ARMED) setState(RS_ARMED_IDLE, "FC_CONFIRMED_ARM");
    else if (pl[0] == FC_DISARMED || pl[1] != 0) { armLatchClear = false; setState(RS_DISARMED, pl[1] ? rejName(pl[1]) : "ARM_NOT_CONFIRMED"); }
  }
}
static void handleRadioPacket(uint8_t type, uint8_t reqId, const uint8_t *pl, uint8_t plen) {
  uint32_t now = millis();
  int ti = tIdx(type);
  if (ti >= 0) {
    size_t need = 0; void *dst = NULL;
    switch (ti) {
      case T_BASIC: need = sizeof(TBasic); dst = &tb; break; case T_BATT: need = sizeof(TBatt); dst = &tbat; break;
      case T_GPS: need = sizeof(TGps); dst = &tgps; break; case T_SENS: need = sizeof(TSens); dst = &tsens; break;
      case T_FLIGHT: need = sizeof(TFlight); dst = &tfl; break; case T_LINK: need = sizeof(TFcLink); dst = &tlnk; break;
    }
    if (plen == need) { memcpy(dst, pl, need); tc[ti].valid = true; tc[ti].rxMs = now; } else stRxLenBad++;
    return;
  }
  switch (type) {
    case PT_HELLO:
      if (plen >= 7) {
        fcFwMaj = pl[0]; fcFwMin = pl[1]; memcpy(&fcCaps, pl + 2, 4); fcId = pl[6]; capsKnown = true;
        tc[T_BATT].unsupported = !(fcCaps & CAP_T_BATT); tc[T_GPS].unsupported = !(fcCaps & CAP_T_GPS);
        tc[T_SENS].unsupported = !(fcCaps & CAP_T_SENS); tc[T_FLIGHT].unsupported = !(fcCaps & CAP_T_FLIGHT); tc[T_LINK].unsupported = !(fcCaps & CAP_T_LINK);
      } else stRxLenBad++;
      break;
    case PT_PAIR_STATUS: if (plen >= 2) { pairStatus = pl[0]; pairReason = pl[1]; } break;
    case PT_ARM_STATUS: onArmStatus(pl, plen); break;
    case PT_RTH_STATUS:
      if (plen >= 2) { if (rthState != pl[0] || rthReason != pl[1]) emitEvent("RTH_STATUS", ",\"rth_state\":%u,\"reason_code\":%u", pl[0], pl[1]); rthState = pl[0]; rthReason = pl[1]; }
      break;
    case PT_ERROR:
      if (plen >= 3 && reqId == 0) { int t2 = tIdx(pl[0]); if (t2 >= 0 && pl[1] == 1) tc[t2].unsupported = true; }
      break;
    case PT_COMMAND_RESP: case PT_PID_READ_RESP: case PT_PID_WRITE_RESP: case PT_CAL_STATUS: case PT_LOG_RESP: break;
    default: stRxUnknown++; return;
  }
  if (reqId) {
    for (int i = 0; i < NSLOT; i++) {
      Slot &s = slots[i];
      if (!s.used || s.reqId != reqId) continue;
      if (type == PT_ERROR) finishSlot(i, plen >= 2 ? fcErrName(pl[1]) : "FC_ERROR", pl, plen);
      else if (type == s.expect) finishSlot(i, NULL, pl, plen);
      else stRxUnknown++;
      break;
    }
  }
}
static void processRadio() {
  if (!radioReady) return;
  uint32_t now = millis();
  if (!txInFlight) return;
  bool ok, fail, rdy; radio.whatHappened(ok, fail, rdy);
  if (ok || fail) {
    txInFlight = false; stRetries += radio.getARC(); winTx++;
    if (fail) { radio.flush_tx(); stTxFail++; winFail++; } else { stTxOk++; lastAckMs = now; }
    if (winTx >= 100) { lastLossPct = winFail; winTx = 0; winFail = 0; }
    if (txSlot >= 0 && slots[txSlot].used) {
      Slot &s = slots[txSlot];
      if (ok) s.delivered = true; else if (s.tries >= s.maxTries) finishSlot(txSlot, "RADIO_FAIL", NULL, 0);
    }
    txSlot = -1;
    if (ok) {
      uint8_t guard = 3;
      while (guard-- && radio.available()) {
        uint8_t n = radio.getDynamicPayloadSize();
        if (n < 1 || n > 32) { radio.flush_rx(); break; }
        uint8_t buf[32]; radio.read(buf, n);
        uint8_t type, seq, req, plen; const uint8_t *pl;
        if (!validatePacket(buf, n, &type, &seq, &req, &pl, &plen)) continue;
        if (rxHaveSeq) {
          int8_t d = (int8_t)(seq - lastRxSeq);
          if (d <= 0) { stRxDup++; continue; }
          if (d > 1) stRxLost += (uint32_t)(d - 1);
        }
        rxHaveSeq = true; lastRxSeq = seq; lastValidRxMs = now; everConnected = true; stRxValid++;
        lastRpd = radio.testRPD();
        handleRadioPacket(type, req, pl, plen);
      }
    }
  } else if (now - txStartMs > 30) { radio.flush_tx(); txInFlight = false; stTxTimeout++; txSlot = -1; }
}

// ---- Penjadwal pengiriman radio ----
static uint8_t pickWant() {
  static const uint8_t pat[10] = {T_BASIC, T_FLIGHT, T_BASIC, T_BATT, T_BASIC, T_GPS, T_BASIC, T_SENS, T_BASIC, T_LINK};
  static const uint8_t ptype[T_COUNT] = {PT_TELEM_BASIC, PT_TELEM_BATTERY, PT_TELEM_GPS, PT_TELEM_SENSORS, PT_TELEM_FLIGHT, PT_TELEM_LINK};
  for (int k = 0; k < 10; k++) {
    uint8_t t = pat[wantIdx]; wantIdx = (wantIdx + 1) % 10;
    if (!tc[t].unsupported) return ptype[t];
  }
  return PT_TELEM_BASIC;
}
static void buildControl(int16_t out[4], bool &valid) {
  valid = (rstate == RS_FLIGHT) && joyValid && calibrated && fcKnown;
  int16_t thr, yaw, rol, pit;
  if (cfg.stickMode == 2) { thr = axisVal[1]; yaw = axisVal[0]; rol = axisVal[2]; pit = axisVal[3]; }
  else { pit = axisVal[1]; yaw = axisVal[0]; rol = axisVal[2]; thr = axisVal[3]; }
  out[0] = thr; out[1] = yaw; out[2] = rol; out[3] = pit;
  for (int i = 0; i < 4; i++) if (out[i] > 1000 || out[i] < -1000) valid = false;
  if (!valid) for (int i = 0; i < 4; i++) out[i] = 0;
}
static void scheduleRadio() {
  if (!radioReady || txInFlight) return;
  uint32_t now = millis();
  if (now - lastTickMs < (uint32_t)(1000 / cfg.controlHz)) return;
  lastTickMs = now;
  for (int i = 0; i < NSLOT; i++) {
    Slot &s = slots[i];
    if (s.used && !s.delivered && s.tries < s.maxTries && (s.tries == 0 || now - s.lastTx >= 40)) {
      if (sendRadioPacket(s.ptype, s.reqId, s.pl, s.plen)) { s.tries++; s.lastTx = now; txSlot = i; }
      return;
    }
  }
  if (!capsKnown && now - lastHelloMs >= 500) {
    lastHelloMs = now; uint8_t h[5] = {1, PROTO_VER, FW_MAJ, FW_MIN, (uint8_t)rstate};
    sendRadioPacket(PT_HELLO, 0, h, 5); return;
  }
  uint8_t flags = 0;
  int16_t ax[4]; bool cv; buildControl(ax, cv);
  if (cv) flags |= 1; if (rthPressed) flags |= 2; if (calibrated) flags |= 4; if (joyValid) flags |= 8; if (rstate == RS_ARM_GESTURE) flags |= 16;
  uint8_t want = pickWant();
  if (rstate == RS_ARMED_IDLE || rstate == RS_FLIGHT) {
    uint8_t p[10]; memcpy(p, ax, 8); p[8] = flags; p[9] = want; sendRadioPacket(PT_CONTROL, 0, p, 10);
  } else {
    uint8_t p[3] = {(uint8_t)rstate, flags, want}; sendRadioPacket(PT_HEARTBEAT, 0, p, 3);
  }
}

// ======================= PROSES UTAMA =======================
static void onLinkLost() {
  invalidateTelemetry(); capsKnown = false; rxHaveSeq = false;
  failAllSlots("NO_LINK");
  if (rstate == RS_ARM_GESTURE || rstate == RS_ARM_PENDING || rstate == RS_ARMED_IDLE || rstate == RS_FLIGHT) { armLatchClear = false; setState(RS_DISARMED, "LINK_LOST"); }
}
static void processTelemetry() {
  uint32_t now = millis();
  bool alive = lastValidRxMs && (now - lastValidRxMs) < cfg.linkTimeoutMs;
  LinkState ns = !radioReady ? LS_NOT_INIT : (alive ? LS_CONNECTED : (everConnected ? LS_LOST : LS_SEARCHING));
  if (ns != linkState) {
    LinkState prev = linkState; linkState = ns;
    if (ns == LS_LOST && prev == LS_CONNECTED) onLinkLost();
    emitEvent("LINK", ",\"link\":\"%s\"", linkName(ns));
  }
  basicFresh = tc[T_BASIC].valid && (now - tc[T_BASIC].rxMs) <= cfg.telemMaxAgeMs;
  fcKnown = (linkState == LS_CONNECTED) && basicFresh;
}
static void processFailsafe() {
  uint32_t now = millis();
  if (!radioReady && now - lastRadioInitTry > 2000) { lastRadioInitTry = now; initRadio(); }
  if (rstate == RS_FLIGHT && !joyValid) setState(RS_ARMED_IDLE, "JOYSTICK_INVALID");
  if (rstate == RS_FLIGHT && !fcKnown) setState(RS_ARMED_IDLE, "FC_STATE_UNKNOWN");
}
static void processArmGesture() {
  uint32_t now = millis();
  if (fcKnown && capsKnown) {
    uint8_t fs = tb.fcState;
    if ((rstate == RS_ARMED_IDLE || rstate == RS_FLIGHT) && fs == FC_DISARMED) { armLatchClear = false; setState(RS_DISARMED, "FC_DISARMED"); }
    else if ((rstate == RS_DISARMED || rstate == RS_ARM_GESTURE || rstate == RS_ARM_PENDING) && fs == FC_ARMED) setState(RS_ARMED_IDLE, "FC_REPORTS_ARMED");
  }
  if (!calibrated && rstate != RS_CALIBRATING && rstate != RS_CALIBRATION_REQUIRED) setState(RS_CALIBRATION_REQUIRED, "CALIBRATION_LOST");
  uint32_t held = (allNeutral && neutralSince) ? now - neutralSince : 0;
  if (held >= NEUTRAL_ARM_MS) armLatchClear = true;
  switch (rstate) {
    case RS_CALIBRATION_REQUIRED: if (calibrated) setState(RS_DISARMED, "CALIBRATION_OK"); break;
    case RS_CALIBRATING: break;
    case RS_DISARMED:
      if (gestureNow()) {
        if (armLatchClear) {
          const char *why = armBlockReason();
          if (!why) { gStart = now; setState(RS_ARM_GESTURE, "GESTURE_START"); }
          else { lastArmBlock = why; armLatchClear = false; emitEvent("ARM_BLOCKED", ",\"reason\":\"%s\"", why); }
        }
      }
      break;
    case RS_ARM_GESTURE: {
      const char *why = armBlockReason();
      if (!gestureNow() || why) { armLatchClear = false; if (why) lastArmBlock = why; setState(RS_DISARMED, why ? why : "GESTURE_INCOMPLETE"); }
      else if (now - gStart >= cfg.gestureMs) {
        armLatchClear = false;
        uint8_t src = SRC_GESTURE; uint8_t p[1] = {src};
        if (!slotBusy(C_ARM) && enqueueReq(C_ARM, SRC_GESTURE, PT_ARM_REQUEST, PT_ARM_STATUS, p, 1, 1, ARM_PENDING_MS, false, 0, false, NULL, 0) >= 0) { pendStart = now; setState(RS_ARM_PENDING, "ARM_SENT"); }
        else setState(RS_DISARMED, "ARM_ENQUEUE_FAILED");
      }
      break;
    }
    case RS_ARM_PENDING: if (now - pendStart > ARM_PENDING_MS + 200) { armLatchClear = false; setState(RS_DISARMED, "ARM_TIMEOUT"); } break;
    case RS_ARMED_IDLE: if (joyValid && calibrated && fcKnown && held >= NEUTRAL_FLIGHT_MS) setState(RS_FLIGHT, "NEUTRAL_CONFIRMED"); break;
    case RS_FLIGHT: break;
  }
}
static void processRthButton() {
  static bool lastRaw = false, stable = false, latched = false; static uint32_t chg = 0;
  uint32_t now = millis(); bool raw = (digitalRead(PIN_RTH) == LOW);
  if (raw != lastRaw) { lastRaw = raw; chg = now; }
  if (now - chg >= DEBOUNCE_MS && raw != stable) {
    stable = raw; rthPressed = stable;
    if (stable && !latched) {
      latched = true;
      emitEvent("RTH_BUTTON", ",\"pressed\":true");
      const char *why = NULL;
      if (!radioReady) why = "RADIO_NOT_INIT"; else if (linkState != LS_CONNECTED) why = "NO_LINK";
      else if (!capsKnown) why = "FC_NOT_READY"; else if (!(fcCaps & CAP_RTH)) why = "UNSUPPORTED"; else if (slotBusy(C_RTH)) why = "BUSY";
      if (!why) {
        uint8_t p[1] = {SRC_BUTTON};
        if (enqueueReq(C_RTH, SRC_BUTTON, PT_RTH_REQUEST, PT_RTH_STATUS, p, 1, 6, cfg.reqTimeoutMs, false, 0, false, NULL, 0) < 0) why = "BUSY";
      }
      if (why) emitEvent("RTH_RESULT", ",\"accepted\":false,\"reason\":\"%s\"", why);
    } else if (!stable) { latched = false; emitEvent("RTH_BUTTON", ",\"pressed\":false"); }
  }
}
static void processPending() {
  uint32_t now = millis();
  for (int i = 0; i < NSLOT; i++) if (slots[i].used && now - slots[i].qMs > slots[i].timeoutMs) { stReqTimeout++; finishSlot(i, "TIMEOUT", NULL, 0); }
}
static void updateDiagnostics(uint32_t t0us) {
  uint32_t now = millis(), us = micros();
  uint32_t gap = us - lastLoopUs; lastLoopUs = us;
  if (gap > loopMaxUs && gap < 5000000UL) loopMaxUs = gap;
  loopSumUs += (us - t0us); loopCnt++;
  if (now - diagWinMs >= 1000) { diagWinMs = now; loopMaxUsPrev = loopMaxUs; loopAvgUsPrev = loopCnt ? loopSumUs / loopCnt : 0; loopMaxUs = 0; loopSumUs = 0; loopCnt = 0; }
  uint16_t f = 0;
  if (!radioReady) f |= 1; if (!calibrated) f |= 2; if (!joyValid) f |= 4; if (cfgDefaulted) f |= 8; if (usbDrops) f |= 16;
  if (rstate == RS_DISARMED && !allNeutral && !gestureNow() && joyValid && calibrated) { if (!notNeutralSince) notNeutralSince = now; if (now - notNeutralSince > 10000) f |= 32; } else notNeutralSince = 0;
  if (linkState == LS_LOST) f |= 64;
  errFlagsRemote = f;
}

// ======================= JSON KE APK =======================
static void jGroupHead(const char *name, int ti) {
  if (tc[ti].unsupported) { jadd("\"%s\":{\"status\":\"UNSUPPORTED\"", name); return; }
  if (!tc[ti].valid) { jadd("\"%s\":{\"status\":\"NO_DATA\"", name); return; }
  uint32_t age = millis() - tc[ti].rxMs;
  jadd("\"%s\":{\"status\":\"%s\",\"age_ms\":%lu", name, age > cfg.telemMaxAgeMs ? "STALE" : "OK", (unsigned long)age);
}
static bool jHas(int ti) { return tc[ti].valid && !tc[ti].unsupported; }
static void aBasic() {
  jGroupHead("basic", T_BASIC);
  if (jHas(T_BASIC)) {
    jadd(",\"fc_state\":\"%s\",\"flight_mode\":%u,\"preflight\":\"%s\",\"failsafe\":%s,\"rth_state\":%u,\"home_set\":%s,\"flight_time_s\":%u,\"error_flags\":%u,\"arm_reject_code\":%u,\"arm_reject\":\"%s\",\"fc_fw\":\"%u.%u\"",
         fcName(tb.fcState), tb.flightMode, tb.preflight == 1 ? "READY" : (tb.preflight == 0 ? "NOT_READY" : "UNKNOWN"), B(tb.failsafe), tb.rth, B(tb.homeSet),
         tb.flightTimeS, tb.errFlags, tb.armReject, rejName(tb.armReject), tb.fwMaj, tb.fwMin);
  }
  jadd("}");
}
static void aBattery() {
  jGroupHead("battery", T_BATT);
  if (jHas(T_BATT)) {
    jadd(",\"voltage_v\":%.2f", tbat.mV / 1000.0f);
    jN("current_ma", tbat.mA, tbat.mA == 0x7FFF); jN("consumed_mah", tbat.mAh, tbat.mAh == 0xFFFF); jN("percent", tbat.pct, tbat.pct == 255);
    jadd(",\"level\":\"%s\"", tbat.status == 1 ? "OK" : (tbat.status == 2 ? "LOW" : (tbat.status == 3 ? "CRITICAL" : "UNKNOWN")));
  }
  jadd("}");
}
static void aGps() {
  jGroupHead("gps", T_GPS);
  if (jHas(T_GPS)) {
    jadd(",\"fix\":%u,\"sats\":%u", tgps.fix, tgps.sats);
    if (tgps.fix >= 2) jadd(",\"lat\":%.7f,\"lon\":%.7f", (double)tgps.lat * 1e-7, (double)tgps.lon * 1e-7);
    if (tgps.hdop == 0xFFFF) jadd(",\"hdop\":null"); else jadd(",\"hdop\":%.2f", tgps.hdop / 100.0f);
    if (tgps.altDm == 0x7FFF) jadd(",\"alt_gps_m\":null"); else jadd(",\"alt_gps_m\":%.1f", tgps.altDm / 10.0f);
    if (tgps.speedCms == 0xFFFF) jadd(",\"speed_gps_ms\":null"); else jadd(",\"speed_gps_ms\":%.2f", tgps.speedCms / 100.0f);
  }
  jadd("}");
}
static void aFlight() {
  jGroupHead("flight", T_FLIGHT);
  if (jHas(T_FLIGHT)) {
    static const char *src[4] = {"NONE", "BARO", "GPS", "TOF"};
    if (tfl.altDm == 0x7FFF) jadd(",\"altitude_m\":null"); else jadd(",\"altitude_m\":%.1f", tfl.altDm / 10.0f);
    jadd(",\"altitude_source\":\"%s\"", tfl.altSrc < 4 ? src[tfl.altSrc] : "UNKNOWN");
    if (tfl.speedCms == 0xFFFF) jadd(",\"speed_ms\":null"); else jadd(",\"speed_ms\":%.2f", tfl.speedCms / 100.0f);
    jN("home_distance_m", tfl.homeDistM, tfl.homeDistM == 0xFFFF); jN("heading_deg", tfl.headingDeg, tfl.headingDeg == 0x7FFF);
    if (tfl.rollDd == 0x7FFF) jadd(",\"roll_deg\":null"); else jadd(",\"roll_deg\":%.1f", tfl.rollDd / 10.0f);
    if (tfl.pitchDd == 0x7FFF) jadd(",\"pitch_deg\":null"); else jadd(",\"pitch_deg\":%.1f", tfl.pitchDd / 10.0f);
    if (tfl.yawDd == 0x7FFF) jadd(",\"yaw_deg\":null"); else jadd(",\"yaw_deg\":%.1f", tfl.yawDd / 10.0f);
    jN("throttle_cmd_us", tfl.thrCmd, tfl.thrCmd == 0xFFFF);
  }
  jadd("}");
}
static void aSensors() {
  jGroupHead("sensors", T_SENS);
  if (jHas(T_SENS)) {
    static const char *nm[8] = {"imu", "baro", "mag", "gps", "tof", "flow", "current", "voltage"};
    jadd(",\"fc_data_age_ms\":%u,\"error_mask\":%u", tsens.dataAgeMs, tsens.errors);
    for (int i = 0; i < 8; i++) {
      bool p = tsens.present & (1 << i), h = tsens.healthy & (1 << i), c = tsens.calibrated & (1 << i);
      jadd(",\"%s\":{\"state\":\"%s\",\"calibrated\":%s}", nm[i], !p ? "ABSENT" : (h ? "OK" : "FAULT"), B(c));
    }
  }
  jadd("}");
}
static const char *fcConnStr() { return (linkState == LS_CONNECTED && capsKnown) ? "CONNECTED" : (linkState == LS_CONNECTED ? "HANDSHAKE" : "UNKNOWN"); }
static const char *armStateShown() { return fcKnown ? rsName(rstate) : "UNKNOWN"; }
static void aLink() {
  uint32_t now = millis(), since = lastValidRxMs ? now - lastValidRxMs : 0;
  jadd("\"link\":{\"state\":\"%s\",\"ms_since_valid_rx\":", linkName(linkState));
  if (lastValidRxMs) jadd("%lu", (unsigned long)since); else jadd("null");
  jadd(",\"bidirectional\":%s,\"hw_ack_recent\":%s,\"tx\":%lu,\"tx_ok\":%lu,\"tx_fail\":%lu,\"tx_timeout\":%lu,\"retries\":%lu,\"rx_valid\":%lu,\"rx_bad_crc\":%lu,\"rx_dup_old\":%lu,\"rx_lost_est\":%lu,\"rx_unknown\":%lu,\"rx_bad_len\":%lu,\"req_timeouts\":%lu,\"loss_pct_last100\":%lu,\"last_rx_seq\":%u,\"last_tx_seq\":%u,\"rssi\":null,\"rpd_above_64dbm\":%s",
       B(linkState == LS_CONNECTED), B(lastAckMs && now - lastAckMs < cfg.linkTimeoutMs), (unsigned long)stTx, (unsigned long)stTxOk, (unsigned long)stTxFail, (unsigned long)stTxTimeout,
       (unsigned long)stRetries, (unsigned long)stRxValid, (unsigned long)stRxBad, (unsigned long)stRxDup, (unsigned long)stRxLost, (unsigned long)stRxUnknown,
       (unsigned long)stRxLenBad, (unsigned long)stReqTimeout, (unsigned long)lastLossPct, lastRxSeq, txSeq, stRxValid ? B(lastRpd) : "null");
  jadd("}");
  jadd(",");
  jGroupHead("fc_view", T_LINK);
  if (jHas(T_LINK)) jadd(",\"rx_ok\":%u,\"rx_bad\":%u,\"ctrl_gaps\":%u,\"fc_link_state\":%u,\"fc_failsafe\":%s", tlnk.rxOk, tlnk.rxBad, tlnk.ctrlGap, tlnk.linkState, B(tlnk.failsafe));
  jadd("}");
}
static void aRemote() {
  static const char *an[4] = {"lx", "ly", "rx", "ry"};
  jadd("\"joystick\":{\"valid\":%s,\"calibrated\":%s,\"cal_state\":\"%s\",\"cal_loaded\":%s", B(joyValid), B(calibrated), calState == CAL_NONE ? "NONE" : (calState == CAL_CAPTURE ? "CAPTURE_CENTER" : "SWEEP"), B(calLoaded));
  for (int i = 0; i < 4; i++) {
    jadd(",\"%s\":{\"raw\":%u,\"value\":%d,\"adc_ok\":%s", an[i], rawAdc[i], axisVal[i], B(adcOk[i]));
    if (calibrated) jadd(",\"min\":%u,\"center\":%u,\"max\":%u", cal[i].mn, cal[i].ct, cal[i].mx);
    jadd("}");
  }
  jadd(",\"neutral\":%s,\"gesture_active\":%s}", B(allNeutral), B(rstate == RS_ARM_GESTURE));
  jadd(",\"rth_button\":{\"pressed\":%s},\"remote_battery\":{\"supported\":false},\"error_flags\":%u", B(rthPressed), errFlagsRemote);
}

static void respHello(bool hid, uint32_t id) {
  jbeg("HELLO", hid, id);
  jadd(",\"device\":\"NEXEND_REMOTE\",\"protocol\":\"%s\",\"fw\":\"%s\"", PROTO_NAME, FW_STR);
  sendUsbMessage();
}
static void respStatus(bool hid, uint32_t id) {
  jbeg("STATUS", hid, id);
  jadd(",\"usb\":\"CONNECTED\",\"radio\":\"%s\",\"fc\":\"%s\",\"arm_state\":\"%s\",\"remote_state\":\"%s\",\"fc_arm_state\":\"%s\",\"calibrated\":%s,\"arm_block\":", linkName(linkState), fcConnStr(), armStateShown(), rsName(rstate), fcKnown ? fcName(tb.fcState) : "UNKNOWN", B(calibrated));
  const char *w = armBlockReason(); if (w) jadd("\"%s\"", w); else jadd("null");
  jadd(",\"last_arm_block\":\"%s\",\"uptime_ms\":%lu", lastArmBlock, (unsigned long)millis());
  sendUsbMessage();
}

// ======================= PERINTAH DARI APK =======================
static bool fwdCheck(bool hid, uint32_t id, const char *req, uint32_t cap, bool needDisarmed) {
  const char *e = NULL;
  if (!hid) e = "ID_REQUIRED"; else if (!radioReady) e = "RADIO_NOT_INIT"; else if (linkState != LS_CONNECTED) e = "NO_LINK";
  else if (!capsKnown) e = "FC_NOT_READY"; else if (cap && !(fcCaps & cap)) e = "UNSUPPORTED";
  else if (needDisarmed) { if (!fcKnown) e = "FC_STATE_UNKNOWN"; else if (tb.fcState != FC_DISARMED || armedLike() || rstate == RS_ARM_GESTURE) e = "NOT_DISARMED"; else if (calState != CAL_NONE) e = "CALIBRATING"; }
  if (e) { sendError(hid, id, e, req, "permintaan ditolak oleh remote"); return false; }
  return true;
}
static void enqErr(int r, bool hid, uint32_t id, const char *req) {
  if (r == -2) sendError(hid, id, "BUSY", req, "terlalu banyak permintaan identik");
  else if (r < 0) sendError(hid, id, "BUSY", req, "antrean permintaan penuh");
}
static int axisFrom(const char *s) { if (!strcmp(s, "roll")) return 0; if (!strcmp(s, "pitch")) return 1; if (!strcmp(s, "yaw")) return 2; if (!strcmp(s, "altitude") || !strcmp(s, "alt")) return 3; return -1; }

static const char *calAction(const char *act) {
  if (!strcmp(act, "start")) {
    if (!(rstate == RS_DISARMED || rstate == RS_CALIBRATION_REQUIRED)) return "NOT_DISARMED";
    if (fcKnown && tb.fcState != FC_DISARMED) return "FC_NOT_DISARMED";
    if (!joyValid) return "JOYSTICK_INVALID";
    calState = CAL_CAPTURE; calStartMs = millis(); calN = 0; memset(calSum, 0, sizeof(calSum));
    setState(RS_CALIBRATING, "CAL_START"); return NULL;
  }
  if (!strcmp(act, "finish")) {
    if (calState != CAL_SWEEP) return "NOT_IN_SWEEP";
    if (!calValid(calWork)) return "CAL_INVALID";
    if (!saveCalibration(calWork, true)) return "SAVE_FAILED";
    memcpy(cal, calWork, sizeof(cal)); calibrated = true; calLoaded = true; calState = CAL_NONE; setStateCalEnd("CAL_SAVED"); return NULL;
  }
  if (!strcmp(act, "cancel")) { if (calState == CAL_NONE) return "NOT_CALIBRATING"; calState = CAL_NONE; setStateCalEnd("CAL_CANCELLED"); return NULL; }
  if (!strcmp(act, "reset")) {
    if (!(rstate == RS_DISARMED || rstate == RS_CALIBRATION_REQUIRED)) return "NOT_DISARMED";
    if (!saveCalibration(cal, false)) return "SAVE_FAILED";
    calibrated = false; calLoaded = false; setState(RS_CALIBRATION_REQUIRED, "CAL_RESET"); return NULL;
  }
  return "BAD_ACTION";
}

static void cmdConfigGet(bool hid, uint32_t id) {
  jbeg("CONFIG", hid, id);
  jadd(",\"channel\":%u,\"data_rate\":%u,\"pa_level\":%u,\"address\":\"", cfg.channel, cfg.dataRate, cfg.paLevel);
  for (int i = 0; i < 5; i++) jadd("%02X", cfg.addr[i]);
  jadd("\",\"deadzone\":%u,\"invert\":%u,\"filter\":%u,\"gesture_ms\":%u,\"control_hz\":%u,\"stick_mode\":%u,\"apk_arm\":%u,\"link_timeout_ms\":%u,\"req_timeout_ms\":%u,\"telem_max_age_ms\":%u,\"gesture_thr\":%u,\"defaulted\":%s",
       cfg.deadzone, cfg.invert, cfg.filterAlpha, cfg.gestureMs, cfg.controlHz, cfg.stickMode, cfg.apkArm, cfg.linkTimeoutMs, cfg.reqTimeoutMs, cfg.telemMaxAgeMs, cfg.gestureThr, B(cfgDefaulted));
  sendUsbMessage();
}
static int setU(const char *line, const char *key, void *dst, int sz, bool *bad) {
  long v; int r = jNum(line, "payload", key, &v);
  if (r < 0 || (r == 1 && (v < 0 || v > 65535 || (sz == 1 && v > 255)))) { *bad = true; return 0; }
  if (r == 1) { if (sz == 1) *(uint8_t *)dst = (uint8_t)v; else *(uint16_t *)dst = (uint16_t)v; }
  return r;
}
static void cmdConfigSet(const char *line, bool hid, uint32_t id) {
  if (!hid) { sendError(false, 0, "ID_REQUIRED", "SET_CONFIG", "id wajib"); return; }
  if (armedLike() || rstate == RS_ARM_GESTURE || calState != CAL_NONE) { sendError(hid, id, "NOT_DISARMED", "SET_CONFIG", "konfigurasi hanya saat DISARMED"); return; }
  Config c = cfg; bool bad = false, radioChg = false;
  uint8_t ch0 = c.channel, dr0 = c.dataRate, pa0 = c.paLevel, ad0[5]; memcpy(ad0, c.addr, 5);
  setU(line, "channel", &c.channel, 1, &bad); setU(line, "data_rate", &c.dataRate, 1, &bad); setU(line, "pa_level", &c.paLevel, 1, &bad);
  setU(line, "deadzone", &c.deadzone, 2, &bad); setU(line, "invert", &c.invert, 1, &bad); setU(line, "filter", &c.filterAlpha, 1, &bad);
  setU(line, "gesture_ms", &c.gestureMs, 2, &bad); setU(line, "control_hz", &c.controlHz, 1, &bad); setU(line, "stick_mode", &c.stickMode, 1, &bad);
  setU(line, "apk_arm", &c.apkArm, 1, &bad); setU(line, "link_timeout_ms", &c.linkTimeoutMs, 2, &bad); setU(line, "req_timeout_ms", &c.reqTimeoutMs, 2, &bad);
  setU(line, "telem_max_age_ms", &c.telemMaxAgeMs, 2, &bad); setU(line, "gesture_thr", &c.gestureThr, 2, &bad);
  char hx[16]; int ra = jStrV(line, "payload", "address", hx, sizeof(hx));
  if (ra < 0) bad = true;
  if (ra == 1) {
    if (strlen(hx) != 10) bad = true;
    else for (int i = 0; i < 5 && !bad; i++) { char t[3] = {hx[2 * i], hx[2 * i + 1], 0}; char *e; long v = strtol(t, &e, 16); if (*e) bad = true; else c.addr[i] = (uint8_t)v; }
  }
  if (bad || !cfgValid(c)) { sendError(hid, id, "BAD_CONFIG", "SET_CONFIG", "nilai di luar rentang atau format salah; tidak ada yang disimpan"); return; }
  if (!saveConfig(c)) { sendError(hid, id, "SAVE_FAILED", "SET_CONFIG", "gagal menyimpan; konfigurasi lama dipakai"); return; }
  radioChg = (c.channel != ch0 || c.dataRate != dr0 || c.paLevel != pa0 || memcmp(c.addr, ad0, 5) != 0);
  cfg = c; cfgDefaulted = false;
  if (radioChg) { failAllSlots("RADIO_REINIT"); initRadio(); onLinkLost(); lastValidRxMs = 0; everConnected = false; }
  jbeg("CONFIG_RESULT", true, id); jadd(",\"saved\":true,\"radio_reinit\":%s", B(radioChg)); sendUsbMessage();
}

static void processCommand(char *line) {
  const char *v; int vl; bool st;
  if (!jsonBasicValid(line)) { usbParseErr++; sendError(false, 0, "BAD_JSON", "-", "JSON tidak valid"); return; }
  char type[28];
  if (jStrV(line, NULL, "type", type, sizeof(type)) != 1) { usbParseErr++; sendError(false, 0, "MISSING_TYPE", "-", "field type wajib (string)"); return; }
  long idl = 0; int ri = jNum(line, NULL, "id", &idl); bool hid = (ri == 1 && idl >= 0); uint32_t id = hid ? (uint32_t)idl : 0;
  long ver = 1; if (jNum(line, NULL, "version", &ver) == 1 && ver != 1) { sendError(hid, id, "UNSUPPORTED_VERSION", type, "hanya version 1"); return; }
  int u = U_NONE; for (int i = 1; i < U_COUNT; i++) if (!strcmp(type, UCMD[i])) { u = i; break; }
  if (u == U_NONE) { sendError(hid, id, "UNSUPPORTED", type, "tipe pesan tidak dikenal"); return; }
  bool needIdFwd = (u == U_GET_PID || u == U_SET_PID || u == U_CALIBRATE_REQUEST || u == U_GET_FLIGHT_LOGS || u == U_CLEAR_FLIGHT_LOGS || u == U_SET_CONFIG ||
                    u == U_RTH_REQUEST || u == U_ARM_REQUEST || u == U_DISARM_REQUEST || u == U_SET_RTH_ALT || u == U_GET_RTH_ALT);
  if (needIdFwd && !hid) { sendError(false, 0, "ID_REQUIRED", type, "request id wajib"); return; }
  uint32_t T = cfg.reqTimeoutMs;

  switch (u) {
    case U_HELLO: respHello(hid, id); break;
    case U_PING: jbeg("PONG", hid, id); jadd(",\"uptime_ms\":%lu", (unsigned long)millis()); sendUsbMessage(); break;
    case U_GET_STATUS: respStatus(hid, id); break;
    case U_GET_REMOTE_STATUS:
      jbeg("REMOTE_STATUS", hid, id); jadd(",\"device\":\"NEXEND_REMOTE\",\"fw\":\"%s\",\"usb\":\"CONNECTED\",\"radio\":\"%s\",\"remote_state\":\"%s\",\"config_defaulted\":%s,",
                                           FW_STR, linkName(linkState), rsName(rstate), B(cfgDefaulted));
      aRemote(); sendUsbMessage(); break;
    case U_GET_FC_STATUS:
      jbeg("FC_STATUS", hid, id); jadd(",\"connection\":\"%s\",\"caps_known\":%s,\"caps\":%lu,\"fc_id\":%u,\"fc_fw\":\"%u.%u\",\"pair_status\":%u,\"arm_reject_last\":\"%s\",",
                                       fcConnStr(), B(capsKnown), (unsigned long)fcCaps, fcId, fcFwMaj, fcFwMin, pairStatus, rejName(tb.armReject));
      aBasic(); sendUsbMessage(); break;
    case U_GET_TELEMETRY:
      jbeg("TELEMETRY", hid, id); jadd(","); aBasic(); jadd(","); aBattery(); jadd(","); aGps(); jadd(","); aFlight(); jadd(","); aSensors(); sendUsbMessage(); break;
    case U_GET_BATTERY: jbeg("BATTERY", hid, id); jadd(","); aBattery(); sendUsbMessage(); break;
    case U_GET_GPS: jbeg("GPS", hid, id); jadd(","); aGps(); sendUsbMessage(); break;
    case U_GET_FLIGHT_DATA: jbeg("FLIGHT_DATA", hid, id); jadd(","); aFlight(); jadd(","); aBasic(); sendUsbMessage(); break;
    case U_GET_SENSOR_STATUS: jbeg("SENSOR_STATUS", hid, id); jadd(","); aSensors(); sendUsbMessage(); break;
    case U_GET_LINK_STATS: jbeg("LINK_STATS", hid, id); jadd(","); aLink(); sendUsbMessage(); break;
    case U_GET_FLIGHT_MODE:
      jbeg("FLIGHT_MODE", hid, id);
      if (jHas(T_BASIC) && fcKnown) jadd(",\"flight_mode\":%u,\"age_ms\":%lu", tb.flightMode, (unsigned long)(millis() - tc[T_BASIC].rxMs)); else jadd(",\"flight_mode\":null,\"status\":\"UNKNOWN\"");
      sendUsbMessage(); break;
    case U_GET_HOME_POINT: {
      if (hid && linkState == LS_CONNECTED && capsKnown && (fcCaps & CAP_HOME)) {
        uint8_t p[1] = {2}; int r = enqueueReq(C_HOME, SRC_APK, PT_COMMAND_REQ, PT_COMMAND_RESP, p, 1, 3, T, true, id, true, NULL, 0);
        if (r < 0) enqErr(r, hid, id, type);
      } else {
        jbeg("HOME_POINT", hid, id);
        if (fcKnown) jadd(",\"home_set\":%s,\"coordinates\":\"UNSUPPORTED_OR_UNAVAILABLE\"", B(tb.homeSet)); else jadd(",\"home_set\":null,\"status\":\"UNKNOWN\"");
        if (jHas(T_FLIGHT) && tfl.homeDistM != 0xFFFF) jadd(",\"distance_m\":%u", tfl.homeDistM);
        sendUsbMessage();
      }
      break;
    }
    case U_GET_PID: {
      char a[12]; if (jStrV(line, "payload", "axis", a, sizeof(a)) != 1 || axisFrom(a) < 0) { sendError(hid, id, "BAD_PAYLOAD", type, "axis: roll|pitch|yaw|altitude"); break; }
      if (!fwdCheck(hid, id, type, CAP_PID_READ, false)) break;
      uint8_t p[1] = {(uint8_t)axisFrom(a)}; int r = enqueueReq(C_GET_PID, SRC_APK, PT_PID_READ_REQ, PT_PID_READ_RESP, p, 1, 3, T, true, id, true, NULL, 0);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_SET_PID: {
      char a[12]; float kp, ki, kd;
      if (jStrV(line, "payload", "axis", a, sizeof(a)) != 1 || axisFrom(a) < 0 || jFlt(line, "payload", "kp", &kp) != 1 || jFlt(line, "payload", "ki", &ki) != 1 || jFlt(line, "payload", "kd", &kd) != 1 ||
          kp < 0 || ki < 0 || kd < 0 || kp > PID_MAX || ki > PID_MAX || kd > PID_MAX) { sendError(hid, id, "BAD_PAYLOAD", type, "axis,kp,ki,kd wajib; 0..100"); break; }
      if (!fwdCheck(hid, id, type, CAP_PID_WRITE, true)) break;
      uint8_t p[13]; p[0] = (uint8_t)axisFrom(a); memcpy(p + 1, &kp, 4); memcpy(p + 5, &ki, 4); memcpy(p + 9, &kd, 4);
      if (slotBusy(C_SET_PID)) { sendError(hid, id, "BUSY", type, "SET_PID sedang berjalan"); break; }
      int r = enqueueReq(C_SET_PID, SRC_APK, PT_PID_WRITE_REQ, PT_PID_WRITE_RESP, p, 13, 3, T, true, id, false, p, 13);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_CALIBRATE_REQUEST: {
      char tg[12], ac[10];
      if (jStrV(line, "payload", "target", tg, sizeof(tg)) != 1 || jStrV(line, "payload", "action", ac, sizeof(ac)) != 1) { sendError(hid, id, "BAD_PAYLOAD", type, "target dan action wajib"); break; }
      if (!strcmp(tg, "joystick")) {
        const char *e = calAction(ac);
        if (e) sendError(hid, id, e, type, "kalibrasi joystick ditolak");
        else { jbeg("CALIBRATION", true, id); jadd(",\"target\":\"joystick\",\"action\":\"%s\",\"accepted\":true,\"cal_state\":\"%s\"", ac, calState == CAL_NONE ? "NONE" : (calState == CAL_CAPTURE ? "CAPTURE_CENTER" : "SWEEP")); sendUsbMessage(); }
        break;
      }
      int t = !strcmp(tg, "accel") ? 1 : !strcmp(tg, "gyro") ? 2 : !strcmp(tg, "mag") ? 3 : !strcmp(tg, "baro") ? 4 : !strcmp(tg, "level") ? 5 : 0;
      int a = !strcmp(ac, "start") ? 1 : !strcmp(ac, "cancel") ? 2 : !strcmp(ac, "status") ? 3 : 0;
      if (!t || !a) { sendError(hid, id, "BAD_PAYLOAD", type, "target/action tidak dikenal"); break; }
      if (!fwdCheck(hid, id, type, CAP_CALIB, a == 1)) break;
      uint8_t p[2] = {(uint8_t)t, (uint8_t)a}; int r = enqueueReq(C_CALIB, SRC_APK, PT_CAL_REQ, PT_CAL_STATUS, p, 2, 3, T, true, id, a == 3, NULL, 0);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_GET_FLIGHT_LOGS: {
      char op[8]; long idx = 0; if (jStrV(line, "payload", "op", op, sizeof(op)) != 1) strcpy(op, "info");
      if (jNum(line, "payload", "index", &idx) < 0 || idx < 0 || idx > 65535) { sendError(hid, id, "BAD_PAYLOAD", type, "index 0..65535"); break; }
      if (!fwdCheck(hid, id, type, CAP_LOG_READ, false)) break;
      uint8_t p[3] = {(uint8_t)(strcmp(op, "read") == 0 ? 1 : 0), (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8)};
      int r = enqueueReq(C_LOG, SRC_APK, PT_LOG_REQ, PT_LOG_RESP, p, 3, 3, T * 2, true, id, true, NULL, 0);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_CLEAR_FLIGHT_LOGS: {
      bool cf = false; if (jBool(line, "payload", "confirm", &cf) != 1 || !cf) { sendError(hid, id, "BAD_PAYLOAD", type, "payload.confirm=true wajib"); break; }
      if (!fwdCheck(hid, id, type, CAP_LOG_CLEAR, true)) break;
      uint8_t p[3] = {2, 0xEA, 0xC1}; int r = enqueueReq(C_CLEAR_LOG, SRC_APK, PT_LOG_REQ, PT_LOG_RESP, p, 3, 3, T * 2, true, id, false, NULL, 0);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_GET_CONFIG: cmdConfigGet(hid, id); break;
    case U_SET_CONFIG: cmdConfigSet(line, hid, id); break;
    case U_RTH_REQUEST: {
      if (!fwdCheck(hid, id, type, CAP_RTH, false)) break;
      uint8_t p[1] = {SRC_APK}; int r = enqueueReq(C_RTH, SRC_APK, PT_RTH_REQUEST, PT_RTH_STATUS, p, 1, 5, T, true, id, false, NULL, 0);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_ARM_REQUEST: {
      if (!cfg.apkArm) { sendError(hid, id, "APK_ARM_DISABLED", type, "ARM dari APK dinonaktifkan; gunakan gesture stik"); break; }
      if (!fwdCheck(hid, id, type, CAP_ARM, false)) break;
      const char *why = armBlockReason();
      if (!why && rstate != RS_DISARMED) why = "REMOTE_NOT_DISARMED";
      if (!why && !(armLatchClear && allNeutral)) why = "STICKS_NOT_NEUTRAL";
      if (why) { sendError(hid, id, "ARM_BLOCKED", type, why); break; }
      uint8_t p[1] = {SRC_APK}; armLatchClear = false;
      int r = enqueueReq(C_ARM, SRC_APK, PT_ARM_REQUEST, PT_ARM_STATUS, p, 1, 1, ARM_PENDING_MS, true, id, false, NULL, 0);
      if (r < 0) { enqErr(r, hid, id, type); break; }
      pendStart = millis(); setState(RS_ARM_PENDING, "ARM_SENT_APK");
      break;
    }
    case U_DISARM_REQUEST: {
      if (!fwdCheck(hid, id, type, CAP_DISARM, false)) break;
      uint8_t p[1] = {1}; int r = enqueueReq(C_DISARM, SRC_APK, PT_COMMAND_REQ, PT_COMMAND_RESP, p, 1, 3, T, true, id, true, NULL, 0);
      if (r < 0) enqErr(r, hid, id, type);
      break;
    }
    case U_GET_DIAGNOSTICS:
      jbeg("DIAGNOSTICS", hid, id);
      jadd(",\"error_flags\":%u,\"loop_max_us\":%lu,\"loop_avg_us\":%lu,\"usb_drops\":%lu,\"usb_parse_errors\":%lu,\"usb_line_overflow\":%lu,\"usb_truncated\":%lu,\"req_timeouts\":%lu,\"free_heap\":%lu,\"cfg_defaulted\":%s,\"radio_ready\":%s,\"pending_requests\":",
           errFlagsRemote, (unsigned long)loopMaxUsPrev, (unsigned long)loopAvgUsPrev, (unsigned long)usbDrops, (unsigned long)usbParseErr, (unsigned long)usbOverflow, (unsigned long)usbTrunc,
           (unsigned long)stReqTimeout, (unsigned long)ESP.getFreeHeap(), B(cfgDefaulted), B(radioReady));
      { int n = 0; for (int i = 0; i < NSLOT; i++) if (slots[i].used) n++; jadd("%d,", n); }
      aLink(); sendUsbMessage(); break;
    case U_GET_PROTOCOL_INFO:
      jbeg("PROTOCOL_INFO", hid, id);
      jadd(",\"protocol\":\"%s\",\"proto_version\":%d,\"remote_fw\":\"%s\",\"fc_caps_known\":%s,\"fc_caps\":%lu,\"radio_payload_max\":%d,\"supported_requests\":[", PROTO_NAME, PROTO_VER, FW_STR, B(capsKnown), (unsigned long)fcCaps, PKT_MAX_PL);
      for (int i = 1; i < U_COUNT; i++) jadd("%s\"%s\"", i > 1 ? "," : "", UCMD[i]);
      jadd("]"); sendUsbMessage(); break;
  }
  (void)v; (void)vl; (void)st;
}

// ======================= [BIN] PROTOKOL BINER APK (AA 55) =======================
// Frame: [AA][55][MSG_ID][SEQ][LEN][PAYLOAD][CRC_H][CRC_L]; CRC16-CCITT (init FFFF, poly 1021) atas MSG_ID..PAYLOAD.
// Semua field multi-byte BIG-ENDIAN. Mode dipilih otomatis: byte 0xAA di awal baris = biner, selain itu = JSON Lines.
#define BIN_SYNC1 0xAA
#define BIN_SYNC2 0x55
#define BIN_MAXPL 64
enum : uint8_t {
  MSG_HELLO = 0x01, MSG_HEARTBEAT = 0x02, MSG_TELEMETRY = 0x10, MSG_IMU = 0x11, MSG_BARO = 0x12, MSG_GPS = 0x13,
  MSG_BATTERY = 0x14, MSG_SENSOR_HEALTH = 0x15, MSG_RC_CHANNELS = 0x20, MSG_PID_READ_REQ = 0x30, MSG_PID_DATA = 0x31,
  MSG_PID_WRITE_REQ = 0x32, MSG_PID_WRITE_ACK = 0x33, MSG_COMMAND_REQ = 0x40, MSG_COMMAND_ACK = 0x41
};
static uint8_t binSeqOut = 0;
static uint32_t binRxOk = 0, binRxBad = 0, binIgnored = 0;

static void binSend(uint8_t msg, uint8_t seq, const uint8_t *pl, uint8_t len) {
  uint8_t f[7 + BIN_MAXPL]; if (len > BIN_MAXPL) return;
  f[0] = BIN_SYNC1; f[1] = BIN_SYNC2; f[2] = msg; f[3] = seq; f[4] = len;
  if (len) memcpy(f + 5, pl, len);
  uint16_t c = crc16(f + 2, 3 + len); f[5 + len] = c >> 8; f[6 + len] = c & 0xFF;
  usbWrite((const char *)f, 7 + len);
}
static uint8_t *wU16(uint8_t *p, uint16_t v) { *p++ = v >> 8; *p++ = v & 0xFF; return p; }
static uint8_t *wU32(uint8_t *p, uint32_t v) { *p++ = v >> 24; *p++ = v >> 16; *p++ = v >> 8; *p++ = v; return p; }
static uint8_t *wF32(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); return wU32(p, u); }
static uint32_t rU32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static float rF32(const uint8_t *p) { uint32_t u = rU32(p); float f; memcpy(&f, &u, 4); return f; }
static bool tFresh(int ti) { return tc[ti].valid && !tc[ti].unsupported && (millis() - tc[ti].rxMs) <= cfg.telemMaxAgeMs; }

static void binError(uint32_t id, const char *code) {            // COMMAND_ACK: [reqMsg][0xFF][ascii kode]
  uint8_t p[24]; p[0] = (id >> 8) & 0xFF; p[1] = 0xFF; uint8_t n = 2;
  for (const char *c = code; *c && n < 22; c++) p[n++] = (uint8_t)*c;
  binSend(MSG_COMMAND_ACK, id & 0xFF, p, n);
}
static bool binRespond(Slot &s, uint32_t id, const uint8_t *pl, uint8_t plen) {
  uint8_t p[32], *q = p; uint8_t seq = id & 0xFF;
  switch (s.cmd) {
    case C_GET_PID: {          // PID_DATA: [axis][status][kp f32][ki f32][kd f32][cfg_crc u16]
      if (plen < 16) return false; float kp, ki, kd; uint16_t cc; memcpy(&kp, pl + 2, 4); memcpy(&ki, pl + 6, 4); memcpy(&kd, pl + 10, 4); memcpy(&cc, pl + 14, 2);
      *q++ = pl[0]; *q++ = pl[1]; q = wF32(q, kp); q = wF32(q, ki); q = wF32(q, kd); q = wU16(q, cc);
      binSend(MSG_PID_DATA, seq, p, q - p); return true; }
    case C_SET_PID: {          // PID_WRITE_ACK: [axis][result 0=ditolak 1=RAM 2=tersimpan][reason][kp][ki][kd][cfg_crc]
      if (plen < 17) return false; float kp, ki, kd; uint16_t cc; memcpy(&kp, pl + 3, 4); memcpy(&ki, pl + 7, 4); memcpy(&kd, pl + 11, 4); memcpy(&cc, pl + 15, 2);
      *q++ = pl[0]; *q++ = pl[1]; *q++ = pl[2]; q = wF32(q, kp); q = wF32(q, ki); q = wF32(q, kd); q = wU16(q, cc);
      binSend(MSG_PID_WRITE_ACK, seq, p, q - p); return true; }
    case C_RTHALT: {           // [RA] COMMAND_ACK: [0x38 set | 0x39 get][status 0=OK][alt m hi][alt m lo] (big-endian)
      if (plen < 5) return false; uint16_t alt = (uint16_t)pl[3] | ((uint16_t)pl[4] << 8);
      *q++ = (s.plen >= 3) ? 0x38 : 0x39; *q++ = pl[1]; q = wU16(q, alt);
      binSend(MSG_COMMAND_ACK, seq, p, q - p); return true; }
    case C_ARM:   if (plen < 2) return false; *q++ = 1; *q++ = ((pl[0] == FC_ARMED || pl[0] == FC_ARMING) && pl[1] == 0) ? 1 : 0; *q++ = pl[0]; *q++ = pl[1]; binSend(MSG_COMMAND_ACK, seq, p, 4); return true;
    case C_DISARM: if (plen < 3) return false; *q++ = 2; *q++ = (pl[1] == 0) ? 1 : 0; *q++ = pl[1]; *q++ = pl[2]; binSend(MSG_COMMAND_ACK, seq, p, 4); return true;
    case C_RTH:   if (plen < 2) return false; *q++ = 3; *q++ = (pl[0] == 2) ? 1 : 0; *q++ = pl[0]; *q++ = pl[1]; binSend(MSG_COMMAND_ACK, seq, p, 4); return true;
  }
  return false;
}
static void binRequestViaJson(uint8_t msg, uint8_t seq, const uint8_t *pl, uint8_t len) {
  // Gunakan jalur JSON yang sudah tervalidasi penuh; id diberi flag BIN supaya balasan keluar dalam bentuk biner.
  uint32_t id = BIN_FLAG | ((uint32_t)msg << 8) | seq; char ln[160];
  if (msg == MSG_PID_READ_REQ && len >= 1 && pl[0] < 4)
    snprintf(ln, sizeof(ln), "{\"type\":\"GET_PID\",\"id\":%lu,\"payload\":{\"axis\":\"%s\"}}", (unsigned long)id, axisName(pl[0]));
  else if (msg == MSG_PID_WRITE_REQ && len >= 13 && pl[0] < 4) {
    float kp = rF32(pl + 1), ki = rF32(pl + 5), kd = rF32(pl + 9);
    if (!(kp >= 0 && ki >= 0 && kd >= 0)) { binError(id, "BAD_PAYLOAD"); return; }   // NaN/negatif
    snprintf(ln, sizeof(ln), "{\"type\":\"SET_PID\",\"id\":%lu,\"payload\":{\"axis\":\"%s\",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f}}", (unsigned long)id, axisName(pl[0]), kp, ki, kd);
  } else if (msg == MSG_COMMAND_REQ && len >= 3 && pl[0] == 0x38)      // [RA] SET_RTH_ALT dari APK
    snprintf(ln, sizeof(ln), "{\"type\":\"SET_RTH_ALT\",\"id\":%lu,\"payload\":{\"alt_m\":%u}}", (unsigned long)id, (unsigned)(((uint16_t)pl[1] << 8) | pl[2]));
  else if (msg == MSG_COMMAND_REQ && len >= 1 && pl[0] == 0x39)        // [RA] GET_RTH_ALT dari APK
    snprintf(ln, sizeof(ln), "{\"type\":\"GET_RTH_ALT\",\"id\":%lu}", (unsigned long)id);
  else if (msg == MSG_COMMAND_REQ && len >= 1 && pl[0] >= 1 && pl[0] <= 3)
    snprintf(ln, sizeof(ln), "{\"type\":\"%s\",\"id\":%lu}", pl[0] == 1 ? "ARM_REQUEST" : (pl[0] == 2 ? "DISARM_REQUEST" : "RTH_REQUEST"), (unsigned long)id);
  else { binError(id, "BAD_PAYLOAD"); return; }
  processCommand(ln);
}
static void binHandle(uint8_t msg, uint8_t seq, const uint8_t *pl, uint8_t len) {
  uint8_t p[40], *q = p; binRxOk++;
  switch (msg) {
    case MSG_HELLO:       // [proto][fwMaj][fwMin][link][capsKnown][fcMaj][fcMin][caps u32]
      *q++ = PROTO_VER; *q++ = FW_MAJ; *q++ = FW_MIN; *q++ = (uint8_t)linkState; *q++ = capsKnown ? 1 : 0; *q++ = fcFwMaj; *q++ = fcFwMin; q = wU32(q, fcCaps);
      binSend(MSG_HELLO, seq, p, q - p); break;
    case MSG_HEARTBEAT:   // [remoteState][link][fcKnown][fcState][uptime_ms u32]
      *q++ = (uint8_t)rstate; *q++ = (uint8_t)linkState; *q++ = fcKnown ? 1 : 0; *q++ = fcKnown ? tb.fcState : 0; q = wU32(q, millis());
      binSend(MSG_HEARTBEAT, seq, p, q - p); break;
    case MSG_TELEMETRY: { // [valid bit0=basic bit1=flight][basic: fcState,mode,preflight,failsafe,rth,homeSet,flightTimeS u16,errFlags u16,armReject]
                          // [flight: altDm i16,speedCms u16,homeDistM u16,heading i16,rollDd i16,pitchDd i16,yawDd i16,thrCmd u16]
      bool vb = tFresh(T_BASIC), vf = tFresh(T_FLIGHT); *q++ = (vb ? 1 : 0) | (vf ? 2 : 0);
      *q++ = tb.fcState; *q++ = tb.flightMode; *q++ = tb.preflight; *q++ = tb.failsafe; *q++ = tb.rth; *q++ = tb.homeSet; q = wU16(q, tb.flightTimeS); q = wU16(q, tb.errFlags); *q++ = tb.armReject;
      q = wU16(q, (uint16_t)tfl.altDm); q = wU16(q, tfl.speedCms); q = wU16(q, tfl.homeDistM); q = wU16(q, (uint16_t)tfl.headingDeg);
      q = wU16(q, (uint16_t)tfl.rollDd); q = wU16(q, (uint16_t)tfl.pitchDd); q = wU16(q, (uint16_t)tfl.yawDd); q = wU16(q, tfl.thrCmd);
      binSend(MSG_TELEMETRY, seq, p, q - p); break; }
    case MSG_BATTERY:     // [valid][mV u16][mA i16][mAh u16][pct][status 1=OK 2=LOW 3=CRIT]
      *q++ = tFresh(T_BATT) ? 1 : 0; q = wU16(q, tbat.mV); q = wU16(q, (uint16_t)tbat.mA); q = wU16(q, tbat.mAh); *q++ = tbat.pct; *q++ = tbat.status;
      binSend(MSG_BATTERY, seq, p, q - p); break;
    case MSG_GPS:         // [valid][fix][sats][lat i32][lon i32][hdop*100 u16][altDm i16][speedCms u16]
      *q++ = tFresh(T_GPS) ? 1 : 0; *q++ = tgps.fix; *q++ = tgps.sats; q = wU32(q, (uint32_t)tgps.lat); q = wU32(q, (uint32_t)tgps.lon);
      q = wU16(q, tgps.hdop); q = wU16(q, (uint16_t)tgps.altDm); q = wU16(q, tgps.speedCms);
      binSend(MSG_GPS, seq, p, q - p); break;
    case MSG_SENSOR_HEALTH: // [valid][present u16][healthy u16][calibrated u16][errors u16][dataAgeMs u16]; bit: imu,baro,mag,gps,tof,flow,current,voltage
      *q++ = tFresh(T_SENS) ? 1 : 0; q = wU16(q, tsens.present); q = wU16(q, tsens.healthy); q = wU16(q, tsens.calibrated); q = wU16(q, tsens.errors); q = wU16(q, tsens.dataAgeMs);
      binSend(MSG_SENSOR_HEALTH, seq, p, q - p); break;
    case MSG_PID_READ_REQ: case MSG_PID_WRITE_REQ: case MSG_COMMAND_REQ:
      binRequestViaJson(msg, seq, pl, len); break;
    default: binIgnored++; break;   // IMU/BARO/RC_CHANNELS: remote tidak punya datanya
  }
  (void)len; (void)pl;
}
// Parser byte-per-byte. Kembalikan true bila byte dikonsumsi sebagai bagian paket biner.
static bool binFeed(uint8_t c, uint32_t now) {
  static uint8_t st = 0, msg, seq, len, n, buf[BIN_MAXPL + 2]; static uint32_t t0 = 0;
  if (st && now - t0 > 200) st = 0;                       // paket menggantung -> buang
  switch (st) {
    case 0: if (c == BIN_SYNC1 && !(lineLen > 0 && lineBuf[0] == '{')) {   // bukan di tengah baris JSON
              lineLen = 0; lineOver = false; lineStartMs = 0;                      // buang sampah sebelum paket
              st = 1; t0 = now; return true; } return false;
    case 1: if (c == BIN_SYNC2) st = 2; else st = (c == BIN_SYNC1) ? 1 : 0; return true;
    case 2: msg = c; st = 3; return true;
    case 3: seq = c; st = 4; return true;
    case 4: len = c; n = 0; if (len > BIN_MAXPL) { binRxBad++; st = 0; } else st = len ? 5 : 6; return true;
    case 5: buf[n++] = c; if (n >= len) st = 6; return true;
    case 6: buf[len] = c; st = 7; return true;           // CRC_H
    case 7: {                                              // CRC_L
      uint8_t h[3] = {msg, seq, len}; uint16_t cc = 0xFFFF;
      uint8_t all[3 + BIN_MAXPL]; memcpy(all, h, 3); memcpy(all + 3, buf, len); cc = crc16(all, 3 + len);
      st = 0;
      if (cc == (((uint16_t)buf[len] << 8) | c)) { usbBinary = true; binHandle(msg, seq, buf, len); } else binRxBad++;
      return true; }
  }
  st = 0; return false;
}

static void processUsbSerial() {
  uint32_t now = millis(); uint8_t budget = 96;
  while (budget-- && Serial.available() > 0) {
    int c = Serial.read(); if (c < 0) break;
    lastUsbRxMs = now;
    if (binFeed((uint8_t)c, now)) continue;
    if (c == '\n') {
      if (lineOver) { usbOverflow++; sendError(false, 0, "LINE_TOO_LONG", "-", "baris melebihi 256 byte"); }
      else if (lineLen) { lineBuf[lineLen] = 0; usbBinary = false; processCommand(lineBuf); }
      lineLen = 0; lineOver = false; lineStartMs = 0; continue;
    }
    if (c == '\r') continue;
    if (lineLen == 0 && !lineOver) lineStartMs = now;
    if (lineLen < LINE_MAX) lineBuf[lineLen++] = (char)c; else lineOver = true;
  }
  if ((lineLen > 0 || lineOver) && lineStartMs && now - lineStartMs > 1500) {
    lineLen = 0; lineOver = false; lineStartMs = 0; usbTrunc++;
    sendError(false, 0, "TRUNCATED", "-", "baris tidak selesai dalam 1500 ms, dibuang");
  }
}

// ======================= SETUP & LOOP =======================
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);         // jangan blok kalau USB tidak terhubung
  EEPROM.begin(256);
  memset(slots, 0, sizeof(slots)); memset(tc, 0, sizeof(tc));
  loadConfig(); loadCalibration();
  pinMode(PIN_RTH, INPUT_PULLUP);
  initJoystick();
  initRadio();
  rstate = calibrated ? RS_DISARMED : RS_CALIBRATION_REQUIRED; stateSinceMs = millis();
  lastLoopUs = micros();
}

void loop() {
  uint32_t t0 = micros();
  readJoystick();
  calibrateJoystick();
  processRadio();
  processUsbSerial();
  processRthButton();
  processTelemetry();
  processFailsafe();
  processArmGesture();
  processPending();
  scheduleRadio();
  updateDiagnostics(t0);
}
