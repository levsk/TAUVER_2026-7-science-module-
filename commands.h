#pragma once
// ============================================================================
//  commands.h — "MESSAGES BETWEEN THE BOXES."  (contract v3)
//  The station GUI and the Jetson bridge speak the SAME text commands; nothing
//  object-like crosses the cable. Every line gets exactly one {"type":"ack"}
//  back, so the sender always knows whether its command landed and why not.
//
//  Grammar:  PREFIX:VERB[=ARG[,ARG...]]   (case-insensitive, one per line)
//  Permission classes:
//    ANY     always accepted (stop, status, confirm, external state, reads)
//    IDLE    no mission running and not LOCKed
//    MANUAL  mode MANUAL, no mission running, not LOCKed
//    CAL     MANUAL + nothing moving (writes calibration to flash)
// ============================================================================
#include "mission.h"

// The registry: every sensor, in the order its fields appear.
Subsystem* const FAST_SUBS[] = { &imu, &scaleRock, &scaleReg, &scaleDeep, &phProbe, &refl, &liquid };   // 10 Hz fields
Subsystem* const SLOW_SUBS[] = { &met, &co2, &wind, &liqTemp, &swir };                                  // 2 Hz fields
Subsystem* const SUBS[] = { &imu, &scaleRock, &scaleReg, &scaleDeep, &met, &co2, &wind, &liqTemp, &phProbe, &liquid, &refl, &swir };
Subsystem* findSub(const char* n) { for (Subsystem* s : SUBS) if (!strcasecmp(n, s->name)) return s; return nullptr; }

// --------------------------------------------------------------- handlers ---
typedef bool (*Handler)(char* arg, const char*& msg);
enum Perm : uint8_t { P_ANY, P_IDLE, P_MANUAL, P_CAL };
struct Command { const char* verb; Handler fn; Perm perm; const char* help; };

uint8_t splitArgs(char* a, char** out, uint8_t max) {    // split "A,B,C" in place; returns count
  uint8_t n = 0; if (!a || !*a) return 0;
  out[n++] = a;
  for (char* p = a; *p && n < max; p++) if (*p == ',') { *p = 0; out[n++] = p + 1; }
  return n;
}
bool parseBool(const char* a, bool& v) { if (!strcmp(a, "1") || !strcmp(a, "ON") || !strcmp(a, "TRUE")) { v = true; return true; } if (!strcmp(a, "0") || !strcmp(a, "OFF") || !strcmp(a, "FALSE")) { v = false; return true; } return false; }
uint8_t groupMask(const char* s) {                       // DARK = 0, VIS = 1, NIR = 2, ALL = 3, 0xFF = unknown
  if (!s) return 0xFF;
  if (!strcmp(s, "DARK")) return 0;
  if (!strcmp(s, "ALL"))  return (uint8_t)((1 << N_ILLUM) - 1);
  for (uint8_t g = 0; g < N_ILLUM; g++) if (!strcasecmp(s, ILLUM_NAME[g])) return (uint8_t)(1 << g);
  return 0xFF;
}

void sendHealth();
void sendBoot(const char* reset, bool calOk);

bool cPing(char*, const char*& m)      { J.begin("pong").kvu("ms", millis()).kvs("fw", FW_VERSION); emit(); m = "PONG"; return true; }
bool cStatus(char*, const char*& m)    { sendHealth(); m = "HEALTH_SENT"; return true; }
bool cSelftest(char* a, const char*& m) {
  bool motion = !strcmp(a, "MOTION");
  if (*a && !motion)                       { m = "USE CMD:SELFTEST or CMD:SELFTEST=MOTION"; return false; }
  if (motion && g_mode != MODE_MANUAL)     { m = "MOTION_NEEDS_MODE_MANUAL"; return false; }
  if (refl.busy() || swir.busy() || carb.busy()) { m = "BUSY"; return false; }
  if (!selftest.start(motion, m)) return false;
  m = motion ? "SELFTEST_WITH_LID_JOG" : "SELFTEST_STARTED"; return true;
}
bool cEstop(char*, const char*& m)     { enterSafe("OPERATOR_ESTOP"); m = "SAFE_LATCHED"; return true; }
bool cClearSafe(char*, const char*& m) { if (!clearSafe(m)) return false; m = "SAFE_CLEARED"; return true; }
bool cMode(char* a, const char*& m) {
  Mode n;
  if      (!strcmp(a, "MANUAL")) n = MODE_MANUAL;
  else if (!strcmp(a, "AUTO"))   n = MODE_AUTO;
  else if (!strcmp(a, "ROS2"))   n = MODE_ROS2;
  else { m = "USE MANUAL|AUTO|ROS2"; return false; }
  if (seq.running() && n != g_mode) { m = "MISSION_RUNNING_ABORT_FIRST"; return false; }
  g_mode = n; m = MODE_NAME[n]; return true;
}
bool cPolicy(char* a, const char*& m) {
  Policy p;
  if (!strcmp(a, "ASK")) p = POL_ASK; else if (!strcmp(a, "FAST")) p = POL_FAST; else { m = "USE ASK|FAST"; return false; }
  if (seq.running() && p != g_policy) { m = "MISSION_RUNNING_ABORT_FIRST"; return false; }
  g_policy = p; m = POLICY_NAME[p]; return true;
}
bool cLock(char* a, const char*& m) {
  bool v; if (!parseBool(a, v)) { m = "USE 0|1"; return false; }
  if (v && (seq.running() || anythingMoving())) { m = "BUSY_STOP_FIRST"; return false; }
  g_lock = v; m = v ? "LOCKED_READ_ONLY" : "UNLOCKED"; return true;
}
bool cMoving(char* a, const char*& m) { bool v; if (!parseBool(a, v)) { m = "USE 0|1"; return false; } ext.moving.set(v); m = v ? "MOVING" : "STOPPED"; return true; }
bool cBatt(char* a, const char*& m)   { long v = atol(a); if (v < 0 || v > 100 || !*a) { m = "USE 0..100"; return false; } ext.batt = (int16_t)v; ext.battAt = millis(); m = "OK"; return true; }
bool cArm(char* a, const char*& m)    { bool v; if (!parseBool(a, v)) { m = "USE 0|1"; return false; } ext.armClear.set(v); m = v ? "ARM_CLEAR" : "ARM_IN_WORKSPACE"; return true; }
bool cLiquid(char* a, const char*& m) { bool v; if (!parseBool(a, v)) { m = "USE 0|1"; return false; } ext.probeInLiquid.set(v); m = v ? "PROBE_IN_LIQUID" : "PROBE_OUT"; return true; }
bool cDepth(char* a, const char*& m)  { long v = atol(a); if (!*a || v < 0 || v > 2000) { m = "USE 0..2000 (mm)"; return false; } ext.depthMm = v; ext.depthAt = millis(); m = "OK"; return true; }
bool cMissions(char*, const char*& m) { for (uint8_t k = 0; k < N_MISSIONS; k++) emitMissionDef(MISSIONS[k], missionValid[k]); m = "MISSION_DEFS_SENT"; return true; }
bool cStart(char* a, const char*& m)  { if (!*a) { m = "USE CMD:MISSION_START=<name> (CMD:MISSIONS lists them)"; return false; } if (!seq.start(a, m)) return false; m = "MISSION_ARMED_5S"; return true; }
bool cAbort(char*, const char*& m)    { if (!seq.running()) { m = "NO_MISSION"; return false; } seq.abort("OPERATOR_ABORT"); m = "ABORTED"; return true; }
bool cConfirm(char*, const char*& m)  { if (!seq.confirm(true, m)) return false; m = "CONFIRMED"; return true; }
bool cDeny(char*, const char*& m)     { if (!seq.confirm(false, m)) return false; m = "DENIED_STEP_SKIPPED"; return true; }
bool cReset(char* a, const char*& m)  { Subsystem* s = findSub(a); if (!s) { m = "UNKNOWN_SUBSYSTEM"; return false; } if (s->onI2C) Bus::recover(); s->recover(); m = s->name; return true; }
bool cTare(char* a, const char*& m) {
  int8_t b = *a ? bayIndex(a) : -2;
  if (b == -1) { m = "USE ROCK|REGOLITH|DEEP (empty = all)"; return false; }
  bool ok = true;
  for (int8_t k = 0; k < (int8_t)N_BAYS; k++) if ((b == -2 && BAYS[k].exists && BAYS[k].scale->enabled) || b == k) ok &= BAYS[k].scale->startTare();
  m = ok ? "TARE_STARTED_2S" : "SCALE_OFFLINE"; return ok;
}
bool cCalW(char* a, const char*& m) {
  char* v[2]; if (splitArgs(a, v, 2) != 2) { m = "USE CMD:CALW=<bay>,<grams>"; return false; }
  int8_t b = bayIndex(v[0]); float g = atof(v[1]);
  if (b < 0 || !(g > 0)) { m = "BAD_BAY_OR_GRAMS"; return false; }
  if (!BAYS[b].scale->tared())       { m = "TARE_FIRST"; return false; }      // a span without a zero = a wrong counts/gram, silently
  if (!BAYS[b].scale->startSpan(g)) { m = "SCALE_OFFLINE"; return false; }
  m = "SPAN_STARTED_2S"; return true;
}
bool cCal(char* a, const char*& m) {               // generic contract call: CMD:CAL=<sub>,<args>
  char* c = strchr(a, ','); if (!c) { m = "USE CMD:CAL=<sub>,<args>"; return false; }
  *c = 0; Subsystem* s = findSub(a); if (!s) { m = "UNKNOWN_SUBSYSTEM"; return false; }
  return s->calibrate(c + 1, m);
}
bool cLid(char* a, const char*& m) {
  char* v[2]; if (splitArgs(a, v, 2) != 2) { m = "USE CMD:LID=<bay>,OPEN|CLOSE|HOME|STOP"; return false; }
  int8_t b = bayIndex(v[0]); if (b < 0) { m = "BAD_BAY"; return false; }
  StepperLid* l = BAYS[b].lid;
  if (!strcmp(v[1], "STOP"))  { l->stop(); m = "STOPPED"; return true; }
  // the analysis lid carries the probe and seals the CO2 chamber: it stays shut while either is measuring
  if (b == (int8_t)ANALYSIS_BAY && strcmp(v[1], "CLOSE") && (!strcmp(refl.procName(), "WHITE") || !strcmp(refl.procName(), "SCAN") || swir.busy() || carb.busy()))
    { m = "MEASURING_LID_STAYS_CLOSED"; return false; }
  bool ok = !strcmp(v[1], "OPEN") ? l->open(m) : !strcmp(v[1], "CLOSE") ? l->close(m) : !strcmp(v[1], "HOME") ? l->home(m) : (m = "BAD_ACTION", false);
  if (ok) m = "MOVING"; return ok;
}
// ---- reflectance (the analysis bay) ----
bool cWhite(char* a, const char*& m) {
  if (!strcmp(a, "CLEAR")) { if (refl.busy()) { m = "BUSY"; return false; } refl.clearWhite(); m = "WHITE_CLEARED"; return true; }
  if (*a) { m = "USE CMD:WHITE or CMD:WHITE=CLEAR"; return false; }
  if (selftest.busy()) { m = "SELFTEST_RUNNING"; return false; }
  if (!startWhite(ANALYSIS_BAY, m)) return false; m = "WHITE_STARTED"; return true;
}
bool cReflect(char* a, const char*& m) {
  if (selftest.busy()) { m = "SELFTEST_RUNNING"; return false; }
  if (!startReflect(ANALYSIS_BAY, *a ? a : "MANUAL", m)) return false; m = "REFLECT_STARTED"; return true;
}
bool cLive(char* a, const char*& m) {              // =DARK|VIS|NIR|ALL[,s]  ·  =OFF
  if (!strcmp(a, "OFF")) { if (!strcmp(refl.procName(), "LIVE")) refl.abort("LIVE_OFF"); m = "LIVE_OFF"; return true; }
  char* v[2]; uint8_t n = splitArgs(a, v, 2); uint8_t mask = n ? groupMask(v[0]) : 0xFF;
  if (mask == 0xFF) { m = "USE CMD:LIVE=DARK|VIS|NIR|ALL[,seconds] or =OFF"; return false; }
  if (selftest.busy()) { m = "SELFTEST_RUNNING"; return false; }
  uint32_t ms = n > 1 ? (uint32_t)(atof(v[1]) * 1000.0f) : 0;
  if (!refl.startLive(mask, ms, m)) return false; m = "LIVE_STARTED"; return true;
}
bool cLamp(char* a, const char*& m) {              // =VIS|NIR|ALL,ON[,ms]  ·  =OFF
  if (!strcmp(a, "OFF")) { if (!strcmp(refl.procName(), "LAMP")) refl.abort("LAMP_OFF"); illum.off(); m = "LAMP_OFF"; return true; }
  char* v[3]; uint8_t n = splitArgs(a, v, 3); uint8_t mask = n ? groupMask(v[0]) : 0xFF;
  if (mask == 0xFF || mask == 0 || n < 2 || strcmp(v[1], "ON")) { m = "USE CMD:LAMP=VIS|NIR|ALL,ON[,ms] or =OFF"; return false; }
  if (selftest.busy()) { m = "SELFTEST_RUNNING"; return false; }
  uint32_t ms = n > 2 ? (uint32_t)atol(v[2]) : 0;
  if (!refl.lampTest(mask, ms, m)) return false;
  m = (ms == 0 || ms > REFLECT_LAMP_CAP_MS) ? "LAMP_ON_CAPPED_20S" : "LAMP_ON"; return true;
}
bool cReflectAbort(char*, const char*& m) { refl.abort("OPERATOR"); illum.off(); m = "ABORTED"; return true; }
bool cSwir(char* a, const char*& m) {              // =WHITE | =SCAN[,tag]
  char* v[2]; uint8_t n = splitArgs(a, v, 2);
  if (n) for (char* c = v[0]; *c; c++) *c = (char)toupper((unsigned char)*c);
  if (n < 1 || (strcmp(v[0], "WHITE") && strcmp(v[0], "SCAN"))) { m = "USE CMD:SWIR=WHITE or =SCAN[,tag]"; return false; }
  bool white = !strcmp(v[0], "WHITE");
  if (!startSwir(ANALYSIS_BAY, white, n > 1 ? v[1] : "MANUAL", m)) return false;
  m = white ? "SWIR_WHITE_STARTED" : "SWIR_SCAN_STARTED"; return true;
}
// ---- carbonate, pumps ----
bool cCarb(char* a, const char*& m) {
  int8_t b = *a ? bayIndex(a) : (int8_t)ANALYSIS_BAY;
  if (b < 0) { m = "BAD_BAY"; return false; }
  if (!carb.start((uint8_t)b, m)) return false;
  m = "CARBONATE_STARTED"; return true;
}
bool cCarbAbort(char*, const char*& m) { carb.abort("OPERATOR"); m = "ABORTED"; return true; }
bool cPump(char* a, const char*& m) {               // CMD:PUMP=<name>,FWD|REV,<ms>[,<duty>]
  char* v[4]; uint8_t n = splitArgs(a, v, 4);
  if (n < 3) { m = "USE CMD:PUMP=<acid|sample|rinse>,FWD|REV,<ms>[,<duty>]"; return false; }
  Pump* p = findPump(v[0]); if (!p) { m = "UNKNOWN_PUMP"; return false; }
  Pump::Dir d = !strcmp(v[1], "FWD") ? Pump::FWD : !strcmp(v[1], "REV") ? Pump::REV : Pump::STOP;
  if (d == Pump::STOP) { m = "USE FWD|REV"; return false; }
  if (dose.busy() || carb.busy()) { m = "PROCEDURE_RUNNING"; return false; }
  if (p == &pumpSample && d == Pump::FWD) { m = "SAMPLE_PUMP_FORWARD_ONLY_BY_DOSE"; return false; }   // the 100 mL count must see every mL
  if (!p->run(d, n > 3 ? (uint8_t)constrain(atoi(v[3]), 1, 255) : 0, (uint32_t)atol(v[2]), m)) return false;
  m = "RUNNING"; return true;
}
bool cDose(char* a, const char*& m) {               // CMD:DOSE=<name>,<ml>  (forward, then retract if wired)
  char* v[2]; if (splitArgs(a, v, 2) != 2) { m = "USE CMD:DOSE=<pump>,<ml>"; return false; }
  Pump* p = findPump(v[0]); float ml = atof(v[1]);
  if (!p || !(ml > 0)) { m = "BAD_PUMP_OR_ML"; return false; }
  if (p == &pumpSample && astroMl + ml > ASTRO_MAX_SAMPLE_ML) { m = "WOULD_EXCEED_100ML_RULE"; return false; }
  if (!dose.start(p, ml, p != &pumpRinse, 0, m)) return false;
  m = "DOSING"; return true;
}
bool cPumpStop(char*, const char*& m)  { dose.abort("OPERATOR"); for (Pump* p : PUMPS) p->stop(); m = "ALL_PUMPS_OFF"; return true; }
bool cPumpCal(char* a, const char*& m) {
  char* v[2]; if (splitArgs(a, v, 2) != 2) { m = "USE CMD:PUMP_CAL=<pump>,<ml measured>"; return false; }
  Pump* p = findPump(v[0]); if (!p) { m = "UNKNOWN_PUMP"; return false; }
  if (!p->calibrate(atof(v[1]), m)) return false;
  m = "PUMP_CALIBRATED"; return true;
}
bool cPumpCalRun(char* a, const char*& m) {         // CMD:PUMP_CAL_RUN=<pump>,<ms>: the >= 1 s forward run for calibration (any pump)
  char* v[2]; if (splitArgs(a, v, 2) != 2) { m = "USE CMD:PUMP_CAL_RUN=<pump>,<ms>"; return false; }
  Pump* p = findPump(v[0]); if (!p) { m = "UNKNOWN_PUMP"; return false; }
  uint32_t ms = (uint32_t)atol(v[1]); if (ms < 1000) { m = "RUN_AT_LEAST_1000_MS"; return false; }
  if (!p->run(Pump::FWD, 0, ms, m)) return false;
  m = "RUNNING_MEASURE_THE_VOLUME"; return true;
}
bool cPurge(char* a, const char*& m) {
  uint32_t ms = (uint32_t)atol(a);
  if (ms == 0) { purge.off(); m = "OFF"; return true; }
  if (!purge.on(ms, m)) return false; m = "ON"; return true;
}
bool cPhCal(char* a, const char*& m)  { return phProbe.calibrate(a, m); }
bool cPhMark(char* a, const char*& m) { if (!phProbe.healthy()) { m = "PH_OFFLINE"; return false; } Sequencer::emitPh(*a ? a : "MARK"); m = "PH_RECORDED"; return true; }
bool cBayReset(char* a, const char*& m) {
  if (!strcmp(a, "LIQUID")) { astroMl = 0; m = "NEW_SAMPLE_BOTTLE"; return true; }
  int8_t b = bayIndex(a); if (b < 0) { m = "USE ROCK|REGOLITH|DEEP|LIQUID"; return false; }
  BAYS[b].spent = false; BAYS[b].massOk = false; BAYS[b].lastG = NAN; m = "FRESH_CUP"; return true;
}
bool cImuZero(char*, const char*& m)  { imu.rezero(); m = "HOLD_STILL_1S"; return true; }
bool cMicro(char* a, const char*& m)  { bool v; if (!parseBool(a, v)) { m = "USE ON|OFF"; return false; } if (!v) { microLight.off(); m = "OFF"; return true; } if (!microLight.on(0, m)) return false; m = "ON"; return true; }
bool cAckImage(char* a, const char*& m) { seq.imageAck = *a ? (uint16_t)atoi(a) : seq.imageId; m = "IMAGE_ACKED"; return true; }
bool cHelp(char*, const char*& m);

const Command COMMANDS[] = {
  { "CMD:PING",          cPing,         P_ANY,    "liveness, answers pong" },
  { "CMD:HELP",          cHelp,         P_ANY,    "this list" },
  { "CMD:STATUS",        cStatus,       P_ANY,    "send a health frame now" },
  { "CMD:SELFTEST",      cSelftest,     P_IDLE,   "[=MOTION] plausibility of every module + lamp loop (+ lid jog)" },
  { "CMD:ESTOP",         cEstop,        P_ANY,    "latch SAFE: everything off" },
  { "CMD:OVERRIDE",      cEstop,        P_ANY,    "alias of ESTOP" },
  { "CMD:CLEAR_SAFE",    cClearSafe,    P_ANY,    "leave SAFE (button released)" },
  { "SET:MODE",          cMode,         P_ANY,    "=MANUAL|AUTO|ROS2 who drives" },
  { "SET:POLICY",        cPolicy,       P_ANY,    "=ASK|FAST confirm risky steps, or let sensors decide" },
  { "SET:LOCK",          cLock,         P_ANY,    "=0|1 read-only: every actuator command refused" },
  { "STATE:MOVING",      cMoving,       P_ANY,    "=0|1 rover motion (from ROS)" },
  { "STATE:BATT",        cBatt,         P_ANY,    "=0..100 battery percent (from ROS)" },
  { "STATE:ARM_CLEAR",   cArm,          P_ANY,    "=0|1 arm out of the lid path (from ROS)" },
  { "STATE:PROBE_IN_LIQUID", cLiquid,   P_ANY,    "=0|1 the arm put the probe / intake in the liquid (from ROS)" },
  { "STATE:DEPTH",       cDepth,        P_ANY,    "=<mm> deep-sample depth (from the drill)" },
  { "CMD:MISSIONS",      cMissions,     P_ANY,    "send all mission tables" },
  { "CMD:MISSION_START", cStart,        P_ANY,    "=SURFACE|DEEP|SAMPLING|EXPLORE|ASTROBIO (mode AUTO/ROS2): ARMED 5 s, then runs" },
  { "CMD:MISSION_ABORT", cAbort,        P_ANY,    "stop the mission, everything off, IDLE" },
  { "CMD:CONFIRM",       cConfirm,      P_ANY,    "answer YES to a confirm_request" },
  { "CMD:DENY",          cDeny,         P_ANY,    "answer NO: the step is skipped" },
  { "CMD:RESET",         cReset,        P_ANY,    "=<sub> bus-kick + re-init (also CMD:RESET_<sub>)" },
  { "CMD:TARE",          cTare,         P_CAL,    "[=ROCK|REGOLITH|DEEP] zero a scale (empty tray); empty = all" },
  { "CMD:CALW",          cCalW,         P_CAL,    "=<bay>,<grams> span with a known mass" },
  { "CMD:CAL",           cCal,          P_CAL,    "=<sub>,<args> generic calibrate()" },
  { "CMD:PH_CAL",        cPhCal,        P_CAL,    "=<buffer pH>|CLEAR capture a stable buffer reading" },
  { "CMD:PUMP_CAL_RUN",  cPumpCalRun,   P_CAL,    "=<pump>,<ms> forward run into a measuring cylinder" },
  { "CMD:PUMP_CAL",      cPumpCal,      P_CAL,    "=<pump>,<ml> what that run moved" },
  { "CMD:IMU_ZERO",      cImuZero,      P_CAL,    "re-measure gyro bias (hold still)" },
  { "CMD:LID",           cLid,          P_MANUAL, "=<bay>,OPEN|CLOSE|HOME|STOP" },
  { "CMD:WHITE",         cWhite,        P_MANUAL, "[=CLEAR] white reference on the EMPTY analysis cup (lid closed)" },
  { "CMD:REFLECT",       cReflect,      P_MANUAL, "[=<tag>] reflectance scan of the analysis cup" },
  { "CMD:LIVE",          cLive,         P_MANUAL, "=DARK|VIS|NIR|ALL[,s] stream raw frames · =OFF" },
  { "CMD:LAMP",          cLamp,         P_MANUAL, "=VIS|NIR|ALL,ON[,ms] lamp test (20 s cap) · =OFF" },
  { "CMD:REFLECT_ABORT", cReflectAbort, P_ANY,    "stop LIVE / WHITE / scan / lamp test" },
  { "CMD:SWIR",          cSwir,         P_MANUAL, "=WHITE | =SCAN[,tag] SWIR hydration add-on" },
  { "CMD:CARBONATE",     cCarb,         P_MANUAL, "[=<bay>] HCl test (destructive, lid closed)" },
  { "CMD:CARB_ABORT",    cCarbAbort,    P_ANY,    "stop the acid test" },
  { "CMD:PUMP",          cPump,         P_MANUAL, "=<pump>,FWD|REV,<ms>[,<duty>] (sample forward: use DOSE)" },
  { "CMD:DOSE",          cDose,         P_MANUAL, "=<pump>,<ml> (calibrated pumps only)" },
  { "CMD:PUMP_STOP",     cPumpStop,     P_ANY,    "all pumps off" },
  { "CMD:PURGE",         cPurge,        P_MANUAL, "=<ms> purge fan/valve (0 = off)" },
  { "CMD:PH_MARK",       cPhMark,       P_ANY,    "=<label> record the pH now (e.g. CONTROL)" },
  { "CMD:BAY_RESET",     cBayReset,     P_MANUAL, "=ROCK|REGOLITH|DEEP|LIQUID fresh cup / new bottle" },
  { "CMD:MICRO",         cMicro,        P_MANUAL, "=ON|OFF microscope light" },
  { "CMD:ACK_IMAGE",     cAckImage,     P_ANY,    "[=id] Jetson: microscope frame captured" },
};
constexpr uint8_t N_COMMANDS = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

bool cHelp(char*, const char*& m) {
  J.begin("help").arr("cmds");
  for (uint8_t k = 0; k < N_COMMANDS; k++) J.es(COMMANDS[k].verb);
  J.close(']'); emit(); m = "HELP_SENT"; return true;
}

// ------------------------------------------------------------- dispatcher ---
void handleLine(const char* raw) {
  char line[RX_LINE_MAX + 1];
  strlcpy(line, raw, sizeof line);
  char* p = line; while (*p == ' ' || *p == '\t') p++;              // trim
  char* e = p + strlen(p); while (e > p && (e[-1] == ' ' || e[-1] == '\t')) *--e = 0;
  char echo[RX_LINE_MAX + 1]; strlcpy(echo, p, sizeof echo);
  for (char* c = p; *c; c++) *c = (char)toupper((unsigned char)*c);
  char* arg = strchr(p, '=');
  if (arg) *arg++ = 0; else arg = p + strlen(p);
  // free-text arguments (a scan tag, a pH label) keep the operator's spelling: copy them from the echo
  char argbuf[RX_LINE_MAX + 1];
  if ((!strcmp(p, "CMD:REFLECT") || !strcmp(p, "CMD:PH_MARK") || !strcmp(p, "CMD:SWIR")) && *arg) { strlcpy(argbuf, echo + (arg - p), sizeof argbuf); arg = argbuf; }
  if (!strncmp(p, "CMD:RESET_", 10)) { arg = p + 10; p[9] = 0; }    // legacy CMD:RESET_<sub>
  const Command* cmd = nullptr;
  for (uint8_t k = 0; k < N_COMMANDS; k++) if (!strcmp(p, COMMANDS[k].verb)) { cmd = &COMMANDS[k]; break; }
  const char* msg = "";
  bool ok = false;
  if (!cmd) msg = "UNKNOWN_COMMAND_TRY_CMD:HELP";
  else if (cmd->perm != P_ANY && g_lock)                                msg = "LOCKED_READ_ONLY";
  else if (cmd->perm != P_ANY && seq.running())                         msg = "MISSION_RUNNING";
  else if ((cmd->perm == P_MANUAL || cmd->perm == P_CAL) && g_mode != MODE_MANUAL) msg = "NEEDS_MODE_MANUAL";
  else if (cmd->perm == P_CAL && anythingMoving())                      msg = "CALIBRATE_WHILE_IDLE";
  else ok = cmd->fn(arg, msg);
  J.begin("ack").kvu("ms", millis()).kvs("cmd", echo).kvb("ok", ok).kvs("msg", msg ? msg : "");
  emit();
}

// ---------------------------------------------------------- frames out ------
uint32_t g_teleSeq = 0;
LineReader rx;

void sendTelemetry() {
  // Fast frame (10 Hz): what changes quickly. Mission fields only while a mission is armed or running.
  // Every SLOW_EVERY-th frame adds the slow block: settings, rover state, slow sensors, actuator states.
  bool slow = (g_teleSeq % SLOW_EVERY) == 0;
  J.begin("tel").kvu("seq", g_teleSeq++).kvu("ms", millis()).kvs("state", STATE_NAME[g_state])
   .kvb("steady", systemSteady()).kvb("estop", Safety::estop).kvb("auton", seq.autonomous());
  if (seq.running())
    J.kvs("mission", seq.m->name).kv("step", seq.i).kvs("action", seq.actionName()).kvs("wait", seq.waiting)
     .kvu("flags", seq.flags).kvu("confirm", g_state == ST_CONFIRM ? seq.confirmId : 0).kvf("arm_s", seq.armLeftMs() / 1000.0f, 1);
  J.obj("data");
  for (Subsystem* s : FAST_SUBS) if (s->enabled) s->toJson(J);
  J.close('}');
  if (slow) {
    J.obj("slow").kvs("mode", MODE_NAME[g_mode]).kvs("policy", POLICY_NAME[seq.running() ? seq.policy : g_policy]).kvb("lock", g_lock)
     .kvb("moving", roverMoving()).kv("batt", ext.batt).kvb("arm_clear", armIsClear()).kv("liquid", liquidEvidence()).kvb("host", g_hostLinkUp);
    for (Subsystem* s : SLOW_SUBS) if (s->enabled) s->toJson(J);
    if (ENABLE_PH) J.kvf("ph_t", phProbe.tempC(), 1).kvs("ph_src", phProbe.tempSrc());
    for (StepperLid* l : LIDS) if (l->enabled()) J.kvs(l->name, l->stateName());
    if (ENABLE_PUMPS) J.kvs("acid", pumpAcid.dirName()).kvs("sample", pumpSample.dirName()).kvs("rinse", pumpRinse.dirName()).kvf("astro_ml", astroMl, 1);
    if (ENABLE_PURGE) J.kvb("purge", purge.isOn());
    if (ENABLE_CO2) J.kvs("carb", carb.phase());
    if (ENABLE_MICROSCOPE) J.kvb("micro", microLight.isOn());
    J.close('}');
  }
  emit(false);                                       // telemetry: dropped first if the link is congested
}

void sendHealth() {
  J.begin("health").kv("v", CONTRACT_VERSION).kvu("ms", millis()).kvu("up_s", millis() / 1000).kvs("fw", FW_VERSION)
   .kvs("profile", PROFILE_NAME).obj("sub");
  for (Subsystem* s : SUBS) J.kvs(s->name, HEALTH_NAME[s->health()]);
  J.close('}').obj("err");
  for (Subsystem* s : SUBS) if (s->enabled && s->health() != H_OK && s->lastError()[0]) J.kvs(s->name, s->lastError());
  J.close('}').obj("cal");
  for (uint8_t k = 0; k < N_BAYS; k++) if (BAYS[k].exists) J.kvb(BAYS[k].scale->name, BAYS[k].scale->calibrated());
  J.kvb("ph", phProbe.calibrated()).kvb("acid", pumpAcid.calibrated()).kvb("sample", pumpSample.calibrated()).kvb("rinse", pumpRinse.calibrated())
   .close('}').obj("lids");
  for (StepperLid* l : LIDS) if (l->enabled()) { J.obj(l->name).kvs("st", l->stateName()).kv("pos", l->pos()).kvb("homed", l->homed()).kvs("fault", l->fault() ? l->fault() : "").close('}'); }
  J.close('}').obj("bays");
  for (uint8_t k = 0; k < N_BAYS; k++) if (BAYS[k].exists) { J.obj(BAYS[k].name).kvb("mass_ok", BAYS[k].massOk).kvb("spent", BAYS[k].spent).kvf("last_g", BAYS[k].lastG, 1).close('}'); }
  J.close('}');
  if (ENABLE_REFLECT) J.obj("refl").kvs("chip", refl.chipName()).kv("white_age_s", refl.whiteAgeS()).kvs("proc", refl.procName()).kvu("lamp_on_ms", illum.totalMs()).close('}');
  J.kvf("astro_ml", astroMl, 1).kvf("astro_cap_ml", ASTRO_MAX_SAMPLE_ML, 0).kvu("mission_s", seq.elapsedS())
   .obj("ext").kv("moving_age_s", ext.moving.ageS()).kv("batt_age_s", ext.batt >= 0 ? (int32_t)(since(ext.battAt) / 1000) : -1)
   .kv("arm_age_s", ext.armClear.ageS()).kv("liquid_age_s", ext.probeInLiquid.ageS()).kv("depth_mm", ext.depthMm).close('}')
   .obj("link").kvb("host", g_hostLinkUp).kvu("tx_drop", Tx::drops).kvu("tx_drop_event", Tx::dropsEvent).kvu("tx_bytes", Tx::sent).kvu("rx_overflow", rx.overflows)
   .kvu("loop_max_us", g_loopMaxUs).kvu("i2c_err", Bus::errors).kvu("i2c_recov", Bus::recoveries).kvu("de_late", wind.lateDe()).close('}');
  emit();
  g_loopMaxUs = 0;
}

void sendBoot(const char* reset, bool calOk) {
  J.begin("boot").kv("v", CONTRACT_VERSION).kvs("fw", FW_VERSION).kvs("name", FW_NAME).kvs("profile", PROFILE_NAME)
   .kvs("board", "NUCLEO-G491RE").kvs("reset", reset).kvs("cal", calOk ? "loaded" : "defaults").obj("present")
   .kvb("triad", Bus::probe(ADDR_TRIAD)).kvb("as7341", Bus::probe(ADDR_AS7341)).kvb("scd4x", Bus::probe(ADDR_SCD4X))
   .kvb("mpu6050", Bus::probe(ADDR_MPU)).kvb("bme280", Bus::probe(ADDR_BME280)).close('}').obj("enabled");
  for (Subsystem* s : SUBS) J.kvb(s->name, s->enabled);
  J.kvb("lids", ENABLE_LIDS).kvb("deep_bay", ENABLE_DEEP_BAY).kvb("deep_lid", ENABLE_LIDS && ENABLE_DEEP_LID).kvb("pumps", ENABLE_PUMPS)
   .kvb("purge", ENABLE_PURGE).kvb("microscope", ENABLE_MICROSCOPE).kvb("watchdog", ENABLE_WATCHDOG).kvb("estop_in", ENABLE_ESTOP_IN).close('}')
   .kvf("sample_target_g", SAMPLE_TARGET_G, 0).kvs("analysis_bay", BAYS[ANALYSIS_BAY].name).arr("missions");
  for (uint8_t k = 0; k < N_MISSIONS; k++) J.es(MISSIONS[k].name);
  J.close(']');
  emit();
}
