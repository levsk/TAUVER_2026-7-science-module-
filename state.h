#pragma once
// ============================================================================
//  state.h — who is in charge, what the rover told us, what may move.
//  (Flight only. core.h / subsystem.h / reflect.h are shared with the bench.)
//
//    System state   IDLE -> ARMED (5 s) -> MISSION <-> CONFIRM ;  SAFE from anywhere
//    Mode           MANUAL (GUI drives) · AUTO (GUI starts missions) · ROS2 (Jetson)
//    Policy         ASK (operator confirms risky steps) · FAST (sensors answer, no touches)
//    Lock           read-only: every actuator command refused (report-writing time)
//    Ext            what ROS forwards: moving, battery, arm clear, probe in liquid, depth
//    Safety         the stop button + "may anything move right now?"
//    Cal            scales, pH, pumps: one CRC-checked record in flash
// ============================================================================
#include "core.h"

enum SysState : uint8_t { ST_IDLE, ST_ARMED, ST_MISSION, ST_CONFIRM, ST_SAFE };
enum Mode     : uint8_t { MODE_MANUAL, MODE_AUTO, MODE_ROS2 };
enum Policy   : uint8_t { POL_ASK, POL_FAST };
const char* const STATE_NAME[]  = { "IDLE", "ARMED", "MISSION", "CONFIRM", "SAFE" };
const char* const MODE_NAME[]   = { "MANUAL", "AUTO", "ROS2" };
const char* const POLICY_NAME[] = { "ASK", "FAST" };

SysState g_state  = ST_IDLE;
Mode     g_mode   = MODE_MANUAL;
Policy   g_policy = POL_ASK;
bool     g_lock   = false;
uint32_t g_lastHostRx = 0;
bool     g_hostLinkUp = false;
uint32_t g_loopMaxUs = 0;

// ----------------------------------------------------------------------------
// External state, forwarded by the Jetson as STATE:* lines (or typed on the bench).
// Every value remembers WHEN it arrived. In ROS2 mode an old value is worth
// nothing: no fresh "not moving" = moving, no fresh "arm clear" = not clear.
// ----------------------------------------------------------------------------
struct ExtFlag {
  bool v = false; uint32_t at = 0; bool known = false;
  void set(bool x)                { v = x; at = millis(); known = true; }
  bool fresh(uint32_t ms) const   { return known && since(at) <= ms; }
  int32_t ageS() const            { return known ? (int32_t)(since(at) / 1000) : -1; }
};
struct Ext {
  ExtFlag moving, armClear, probeInLiquid;
  int16_t batt = -1;     uint32_t battAt = 0;   // -1 = unknown
  int32_t depthMm = -1;  uint32_t depthAt = 0;  // deep-sample depth from the drill, -1 = unknown
} ext;

bool roverMoving() {                       // fail safe in ROS2: stale = moving
  if (g_mode == MODE_ROS2) return !ext.moving.fresh(EXT_FRESH_MS) || ext.moving.v;
  return ext.moving.known && ext.moving.v;
}
bool armIsClear() {                        // bench / AUTO: assumed clear unless told otherwise
  if (g_mode == MODE_ROS2) return ext.armClear.fresh(EXT_FRESH_MS) && ext.armClear.v;
  return !ext.armClear.known || ext.armClear.v;
}
int8_t probeInLiquidExt() {                // 1 / 0 from the arm, -1 = nobody told us (recently)
  uint32_t ttl = g_mode == MODE_ROS2 ? EXT_FRESH_MS : EXT_STATE_STALE_MS;
  return ext.probeInLiquid.fresh(ttl) ? (ext.probeInLiquid.v ? 1 : 0) : -1;
}

// ----------------------------------------------------------------------------
// Safety: the stop button. Debounced: 5 ms to trip, 50 ms to release.
// (2.0 also had a UVC lid interlock with a hardware interrupt. Visible and NIR
//  LEDs are not a hazard, so 3.0 stops everything within one loop pass.)
// ----------------------------------------------------------------------------
namespace Safety {
  bool estop = false; uint32_t edge = 0; bool rawLast = false;
  bool estopRaw() {
    if (!ENABLE_ESTOP_IN) return false;
    bool high = digitalRead(PIN_ESTOP) == HIGH;
    return ESTOP_ACTIVE_LOW ? !high : high;
  }
  void init() { if (ENABLE_ESTOP_IN) pinMode(PIN_ESTOP, INPUT_PULLUP); rawLast = estopRaw(); estop = rawLast; edge = millis(); }
  void poll() {
    bool r = estopRaw();
    if (r != rawLast) { rawLast = r; edge = millis(); }
    if (r && !estop && since(edge) >= 5)  estop = true;
    if (!r && estop && since(edge) >= 50) estop = false;
  }
  bool actuationAllowed() { return g_state != ST_SAFE && !estop; }
  const char* whyNot()    { return estop ? "ESTOP" : "SAFE"; }
}

// ----------------------------------------------------------------------------
// Calibration store: one record at offset 0 of the emulated EEPROM page.
// (The reflectance white has its own record at REFLECT_WHITE_STORE_OFF.)
// ----------------------------------------------------------------------------
struct CalData {
  uint32_t magic; uint16_t version, size;     // Store:: header
  int32_t  scaleOffset[N_BAYS];   // raw HX711 counts with an empty tray   [0]=ROCK [1]=REGOLITH [2]=DEEP
  float    scaleCpg[N_BAYS];      // counts per gram, 0 = uncalibrated
  float    phV[2], phP[2];        // two calibration points: module volts and buffer pH
  float    phTcal;                // temperature at calibration (C)
  uint8_t  phN;                   // calibration points captured (2 = calibrated)
  uint8_t  taredMask;             // bit k = bay k has been tared: a span needs a real zero first
  uint8_t  pad[2];
  float    pumpMlps[3];           // mL per second at PUMP_DEFAULT_DUTY: [0]=ACID [1]=SAMPLE [2]=RINSE
  uint32_t crc;                   // Store:: trailer
};
CalData cal;
constexpr uint32_t CAL_MAGIC   = 0x54415633;   // "TAV3"
constexpr uint16_t CAL_VERSION = 3;
constexpr uint16_t CAL_STORE_OFF = 0;
static_assert(sizeof(CalData) <= REFLECT_WHITE_STORE_OFF, "the calibration record would overlap the white");

void calDefaultsPh() {
  cal.phP[0] = 7.0f;  cal.phV[0] = PH_DEFAULT_V7;
  cal.phP[1] = 4.01f; cal.phV[1] = PH_DEFAULT_V7 + PH_DEFAULT_SLOPE * (4.01f - 7.0f);
  cal.phTcal = 25.0f; cal.phN = 0;              // nominal curve, flagged uncalibrated
}
void calDefaults() { memset(&cal, 0, sizeof(cal)); calDefaultsPh(); }
bool calLoad()     { bool ok = Store::load(CAL_STORE_OFF, &cal, sizeof cal, CAL_MAGIC, CAL_VERSION); if (!ok) calDefaults(); return ok; }
void calSave()     { Store::save(CAL_STORE_OFF, &cal, sizeof cal, CAL_MAGIC, CAL_VERSION); }   // ~50 ms of flash work: idle only
