#pragma once
// ============================================================================
//  mission.h — "A TABLE TO DRIVE THE DANCE."
//  A mission is DATA: rows of {action, bay, flags, timeout, parameter}.
//  New lid, new step, new order = edit a table, never the sequencer.
//
//  Five missions, one per ERC science task the payload serves (2026 rules):
//    SURFACE   §7.3.1.2  rock + regolith: collect, close, weigh (>= 100 g at 1 g)
//    DEEP      §7.3.1.3  the drill's sample into the deep bay, weigh, depth logged
//    SAMPLING  §7.3.1.2+3 both in one run (they share the run)
//    EXPLORE   §7.3.1.1  one sample WE chose: white -> weigh -> image -> reflectance -> acid LAST
//    ASTROBIO  §7.3.1.4  pH in the liquid + < 100 mL leak-proof sample
//
//  The only way in:  IDLE -> ARMED (5 s countdown, REQ-OPS-090) -> MISSION.
//  Policy of the run: ASK  = the operator confirms acid and liquid steps
//                     FAST = no questions (REQ-OPS-040: touching the station
//                            forfeits autonomy); sensors and ROS state answer
//                            instead. A preflight with warnings falls back to ASK.
//  Gates, checked in this order for every step:
//    module switched off in config -> skip quietly        bay not fitted -> skip quietly
//    SECONDARY + (clock low | battery low) -> skip        NEEDS_SAMPLE + bay empty -> skip
//    module enabled but dead -> skip + flag HW            CONFIRM (ASK only) -> ask, 60 s silence = DENY
//    STEADY / ARM_CLEAR / LIQUID (FAST only: the arm's report or the tip electrodes) -> wait, timeout -> skip
//  then the action runs, with its own timeout. Nothing ever waits forever.
// ============================================================================
#include "procedures.h"

enum Action : uint8_t { A_PREFLIGHT, A_TARE, A_LID_OPEN, A_LID_CLOSE, A_AWAIT_MASS, A_WEIGH, A_WHITE, A_REFLECT,
                        A_SWIR_WHITE, A_SWIR, A_MICRO_IMAGE, A_CARBONATE, A_PH_MEASURE, A_RINSE, A_INTAKE, A_COMPLETE };
const char* const ACTION_NAME[] = { "PREFLIGHT", "TARE", "LID_OPEN", "LID_CLOSE", "AWAIT_MASS", "WEIGH", "WHITE", "REFLECT",
                                    "SWIR_WHITE", "SWIR", "MICRO_IMAGE", "CARBONATE", "PH_MEASURE", "RINSE", "INTAKE", "COMPLETE" };
// step flags
constexpr uint8_t F_STEADY = 0x01, F_CONFIRM = 0x02, F_SECONDARY = 0x04, F_ARM_CLEAR = 0x08, F_NEEDS_SAMPLE = 0x10, F_LIQUID = 0x20;
// mission flags (accumulated, sent in telemetry "flags")
constexpr uint16_t FL_TIMEOUT = 0x001, FL_SKIPPED = 0x002, FL_DENIED = 0x004, FL_FAILED = 0x008, FL_NO_SAMPLE = 0x010,
                   FL_UNCAL = 0x020, FL_INSUFFICIENT = 0x040, FL_BATT_UNKNOWN = 0x080, FL_HW = 0x100, FL_FALLBACK_ASK = 0x200;

struct MissionStep { Action act; int8_t bay; uint8_t flags; uint16_t timeoutS; float param; };
struct Mission { const char* name; const MissionStep* steps; uint8_t n; uint32_t budgetMs; uint32_t clockLowMs; const char* erc; };

// ------------------------------------------------------------ EDIT HERE ------
//  act             bay    flags                                              timeout  param
const MissionStep M_SURFACE[] = {
  { A_PREFLIGHT,    NOBAY, 0,                                                  10,      0 },
  { A_TARE,         ROCK,  F_STEADY,                                           20,      0 },
  { A_TARE,         REG,   F_STEADY,                                           20,      0 },
  { A_LID_OPEN,     ROCK,  0,                                                  20,      0 },
  { A_AWAIT_MASS,   ROCK,  0,                                                 600,    100 },  // the arm drops the rock: >= 100 g
  { A_LID_CLOSE,    ROCK,  F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        ROCK,  F_STEADY,                                           60,    100 },
  { A_LID_OPEN,     REG,   0,                                                  20,      0 },
  { A_AWAIT_MASS,   REG,   0,                                                 600,    100 },  // the arm pours regolith via the funnel
  { A_LID_CLOSE,    REG,   F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        REG,   F_STEADY,                                           60,    100 },
  { A_COMPLETE,     NOBAY, 0,                                                   0,      0 },
};
const MissionStep M_DEEP[] = {
  { A_PREFLIGHT,    NOBAY, 0,                                                  10,      0 },
  { A_TARE,         DEEP,  F_STEADY,                                           20,      0 },
  { A_LID_OPEN,     DEEP,  0,                                                  20,      0 },
  { A_AWAIT_MASS,   DEEP,  0,                                                 900,    100 },  // the drill pours its 300 mm sample
  { A_LID_CLOSE,    DEEP,  F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        DEEP,  F_STEADY,                                           60,    100 },  // + depth from STATE:DEPTH
  { A_COMPLETE,     NOBAY, 0,                                                   0,      0 },
};
const MissionStep M_SAMPLING[] = {                 // §7.4: surface + deep SHARE one 30 min run, so the waits are shorter
  { A_PREFLIGHT,    NOBAY, 0,                                                  10,      0 },
  { A_TARE,         ROCK,  F_STEADY,                                           20,      0 },
  { A_TARE,         REG,   F_STEADY,                                           20,      0 },
  { A_TARE,         DEEP,  F_STEADY,                                           20,      0 },
  { A_LID_OPEN,     ROCK,  0,                                                  20,      0 },
  { A_AWAIT_MASS,   ROCK,  0,                                                 360,    100 },
  { A_LID_CLOSE,    ROCK,  F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        ROCK,  F_STEADY,                                           60,    100 },
  { A_LID_OPEN,     REG,   0,                                                  20,      0 },
  { A_AWAIT_MASS,   REG,   0,                                                 360,    100 },
  { A_LID_CLOSE,    REG,   F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        REG,   F_STEADY,                                           60,    100 },
  { A_LID_OPEN,     DEEP,  0,                                                  20,      0 },
  { A_AWAIT_MASS,   DEEP,  0,                                                 540,    100 },  // worst case 29.2 of 30 min
  { A_LID_CLOSE,    DEEP,  F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        DEEP,  F_STEADY,                                           60,    100 },
  { A_COMPLETE,     NOBAY, 0,                                                   0,      0 },
};
const MissionStep M_EXPLORE[] = {
  { A_PREFLIGHT,    NOBAY, 0,                                                  10,      0 },
  { A_TARE,         REG,   F_STEADY,                                           20,      0 },
  { A_LID_CLOSE,    REG,   F_ARM_CLEAR,                                        60,      0 },  // the white needs the probe over the cup
  { A_WHITE,        REG,   0,                                                  90,      0 },  // EMPTY cup: the PTFE floor is the white
  { A_SWIR_WHITE,   REG,   F_SECONDARY,                                        30,      0 },
  { A_LID_OPEN,     REG,   0,                                                  20,      0 },
  { A_AWAIT_MASS,   REG,   0,                                                 300,     20 },  // >= 20 g = >= 8 mm deep: optically thick
  { A_LID_CLOSE,    REG,   F_ARM_CLEAR,                                        60,      0 },
  { A_WEIGH,        REG,   F_STEADY | F_NEEDS_SAMPLE,                          60,     20 },
  { A_MICRO_IMAGE,  REG,   F_SECONDARY | F_NEEDS_SAMPLE,                       30,      0 },
  { A_REFLECT,      REG,   F_NEEDS_SAMPLE,                                     90,      0 },
  { A_SWIR,         REG,   F_SECONDARY | F_NEEDS_SAMPLE,                       30,      0 },
  { A_CARBONATE,    REG,   F_STEADY | F_CONFIRM | F_SECONDARY | F_NEEDS_SAMPLE, 330,    0 },  // destructive: LAST
  { A_COMPLETE,     NOBAY, 0,                                                   0,      0 },
};
const MissionStep M_ASTROBIO[] = {
  { A_PREFLIGHT,    NOBAY, 0,                                                  10,      0 },
  { A_RINSE,        NOBAY, F_SECONDARY,                                        30,     10 },  // DI water through the line
  { A_PH_MEASURE,   NOBAY, F_CONFIRM | F_LIQUID,                              180,      0 },  // "is the probe in the liquid?"
  { A_INTAKE,       NOBAY, F_CONFIRM | F_LIQUID,                              180,     50 },  // "is the intake tube in the liquid?"
  { A_RINSE,        NOBAY, F_SECONDARY,                                        30,     10 },
  { A_COMPLETE,     NOBAY, 0,                                                   0,      0 },
};
#define MSN(n, t, b, l, e) { n, t, (uint8_t)(sizeof(t) / sizeof(t[0])), b, l, e }
const Mission MISSIONS[] = {
  MSN("SURFACE",  M_SURFACE,  30UL * 60000, 5UL * 60000, "ERC 7.3.1.2: rock + regolith, stored and weighed onboard (>=100 g at 1 g)"),
  MSN("DEEP",     M_DEEP,     30UL * 60000, 5UL * 60000, "ERC 7.3.1.3: 100 g from 300 mm into the weighed onboard container"),
  MSN("SAMPLING", M_SAMPLING, 30UL * 60000, 5UL * 60000, "ERC 7.3.1.2 + 7.3.1.3 in one run"),
  MSN("EXPLORE",  M_EXPLORE,  20UL * 60000, 4UL * 60000, "ERC 7.3.1.1: in-situ evidence for the registered hypothesis"),
  MSN("ASTROBIO", M_ASTROBIO, 20UL * 60000, 4UL * 60000, "ERC 7.3.1.4: pH + <100 mL leak-proof liquid sample"),
};
constexpr uint8_t N_MISSIONS = sizeof(MISSIONS) / sizeof(MISSIONS[0]);
bool missionValid[N_MISSIONS];
// -----------------------------------------------------------------------------

static bool usesBay(Action a) { return a == A_TARE || a == A_LID_OPEN || a == A_LID_CLOSE || a == A_AWAIT_MASS || a == A_WEIGH ||
                                       a == A_WHITE || a == A_REFLECT || a == A_SWIR_WHITE || a == A_SWIR || a == A_MICRO_IMAGE || a == A_CARBONATE; }
static bool isOptical(Action a) { return a == A_WHITE || a == A_REFLECT || a == A_SWIR_WHITE || a == A_SWIR || a == A_MICRO_IMAGE; }
// Worst case = every step uses its whole timeout + every confirm waits its full 60 s (ASK). Gate waits share the step timer.
uint32_t worstCaseS(const Mission& m) {
  uint32_t t = 0;
  for (uint8_t i = 0; i < m.n; i++) t += m.steps[i].timeoutS + ((m.steps[i].flags & F_CONFIRM) ? CONFIRM_TIMEOUT_MS / 1000 : 0);
  return t;
}

// Boot-time table check, so a typo in a table is caught on the bench, not on the field.
bool validateMission(const Mission& m, const char*& why) {
  if (m.n == 0 || m.steps[m.n - 1].act != A_COMPLETE) { why = "MUST_END_WITH_COMPLETE"; return false; }
  if (worstCaseS(m) * 1000UL > m.budgetMs)            { why = "WORST_CASE_OVER_BUDGET"; return false; }   // must fit the ERC slot
  for (uint8_t i = 0; i < m.n; i++) {
    const MissionStep& s = m.steps[i];
    if (usesBay(s.act) && (s.bay < 0 || s.bay >= (int8_t)N_BAYS)) { why = "STEP_NEEDS_A_BAY"; return false; }
    if (s.act == A_CARBONATE)                                      // acid ruins the sample AND fogs the optics: always last
      for (uint8_t k = i + 1; k < m.n; k++)
        if (isOptical(m.steps[k].act) && m.steps[k].bay == s.bay) { why = "CARBONATE_BEFORE_OPTICS"; return false; }
    if (s.act == A_REFLECT || s.act == A_SWIR) {                   // a scan needs ITS white earlier in the same run
      Action w = s.act == A_REFLECT ? A_WHITE : A_SWIR_WHITE; bool seen = false;
      for (uint8_t k = 0; k < i; k++) if (m.steps[k].act == w && m.steps[k].bay == s.bay) seen = true;
      if (!seen) { why = "SCAN_WITHOUT_WHITE"; return false; }
    }
  }
  return true;
}

void flagNames(uint8_t f, char* out, size_t cap) {
  static const char* const N[] = { "STEADY", "CONFIRM", "SECONDARY", "ARM_CLEAR", "NEEDS_SAMPLE", "LIQUID" };
  out[0] = 0;
  for (uint8_t b = 0; b < 6; b++) if (f & (1 << b)) { if (out[0]) strlcat(out, "|", cap); strlcat(out, N[b], cap); }
}
const char* bayName(int8_t b) { return b >= 0 ? BAYS[b].name : "-"; }
void emitMissionDef(const Mission& m, bool valid) {
  J.begin("mission_def").kvs("name", m.name).kvb("valid", valid).kvu("budget_s", m.budgetMs / 1000)
   .kvu("clock_low_s", m.clockLowMs / 1000).kvu("worst_s", worstCaseS(m)).kvs("erc", m.erc).arr("steps");
  char f[64];
  for (uint8_t i = 0; i < m.n; i++) {
    const MissionStep& s = m.steps[i];
    flagNames(s.flags, f, sizeof f);
    J.eobj().kvs("act", ACTION_NAME[s.act]).kvs("bay", bayName(s.bay)).kvs("flags", f)
     .kvu("timeout_s", s.timeoutS).kvf("param", s.param, 1).close('}');
  }
  J.close(']');
  emit();
}

// "is the probe / intake in the liquid?" FAST mode's answer instead of a click: the arm's own report
// first, then the electrodes at the intake tip. A settled pH reading is NOT evidence: a probe sitting in
// its wet storage cap reads a perfectly stable pH. With neither source, preflight turns the run to ASK.
int8_t liquidEvidence() {
  int8_t e = probeInLiquidExt();
  if (e >= 0) return e;
  if (liquid.enabled && liquid.healthy()) return liquid.wet() ? 1 : 0;
  return -1;
}
bool liquidSourceKnown() { return probeInLiquidExt() >= 0 || (liquid.enabled && liquid.healthy()); }

void enterSafe(const char* why);
bool anythingMoving();

// ============================================================================
class Sequencer {
 public:
  const Mission* m = nullptr;
  uint8_t  i = 0;
  uint16_t flags = 0;
  const char* waiting = "";
  uint16_t confirmId = 0;
  uint16_t imageId = 0, imageAck = 0;
  Policy   policy = POL_ASK;                       // the policy of THIS run

  bool start(const char* name, const char*& why) {
    if (g_lock)                        { why = "LOCKED_READ_ONLY"; return false; }
    if (g_mode == MODE_MANUAL)         { why = "NEEDS_MODE_AUTO_OR_ROS2"; return false; }
    if (g_state == ST_SAFE)            { why = "SAFE_CLEAR_FIRST"; return false; }
    if (m)                             { why = "MISSION_RUNNING"; return false; }
    if (anythingMoving())              { why = "MANUAL_OPERATION_RUNNING"; return false; }
    if (g_mode == MODE_ROS2 && !g_hostLinkUp)                 { why = "ROS2_NEEDS_THE_BRIDGE"; return false; }
    if (g_mode == MODE_ROS2 && !ext.moving.fresh(EXT_FRESH_MS)) { why = "ROS2_NEEDS_FRESH_ROVER_MOTION"; return false; }
    int8_t k = -1;
    for (uint8_t j = 0; j < N_MISSIONS; j++) if (!strcasecmp(name, MISSIONS[j].name)) k = (int8_t)j;
    if (k < 0)                         { why = "UNKNOWN_MISSION"; return false; }
    if (!missionValid[k])              { why = "MISSION_TABLE_INVALID"; return false; }
    m = &MISSIONS[k]; i = 0; flags = 0; policy = g_policy; _confirmed = false; waiting = "";
    _armAt = millis(); g_state = ST_ARMED;
    J.begin("event").kvu("ms", millis()).kvs("lvl", "info").kvs("code", "MISSION_ARMED").kvs("mission", m->name)
     .kvu("arm_s", ARM_DELAY_MS / 1000).kvs("policy", POLICY_NAME[policy]);
    emit();
    return true;
  }
  void abort(const char* why) {
    if (!m) return;
    if (g_state != ST_ARMED) stopAction(m->steps[i]);
    J.begin("event").kvu("ms", millis()).kvs("lvl", "warn").kvs("code", "MISSION_ABORTED").kvs("mission", m->name)
     .kvu("step", i).kvs("why", why).kvu("flags", flags);
    emit();
    m = nullptr; waiting = "";
    if (g_state != ST_SAFE) g_state = ST_IDLE;
  }
  bool confirm(bool yes, const char*& why) {
    if (!m || g_state != ST_CONFIRM) { why = "NOTHING_TO_CONFIRM"; return false; }
    g_state = ST_MISSION;
    if (yes) { _confirmed = true; beginStep(); }
    else     endStep("DENIED", FL_DENIED | FL_SKIPPED);
    return true;
  }
  bool running() const     { return m != nullptr; }
  bool autonomous() const  { return m && policy == POL_FAST && (g_state == ST_ARMED || g_state == ST_MISSION); }
  const char* actionName() const { return m && g_state != ST_ARMED ? ACTION_NAME[m->steps[i].act] : (m ? "ARMED" : "-"); }
  uint32_t elapsedS() const { return m && g_state != ST_ARMED ? since(_t0) / 1000 : 0; }
  int32_t armLeftMs() const { return g_state == ST_ARMED ? (int32_t)ARM_DELAY_MS - (int32_t)since(_armAt) : 0; }

  void service() {
    if (!m || g_state == ST_SAFE) return;
    if (g_state == ST_ARMED) {                     // REQ-OPS-090: nothing moves until 5 s after the command
      if (since(_armAt) < ARM_DELAY_MS) return;
      _t0 = millis(); g_state = ST_MISSION;
      if (!strcmp(m->name, "ASTROBIO")) astroMl = 0;
      J.begin("event").kvu("ms", millis()).kvs("lvl", "info").kvs("code", "MISSION_START").kvs("mission", m->name)
       .kvu("steps", m->n).kvs("policy", POLICY_NAME[policy]);
      emit();
      beginStep();
      return;
    }
    if (g_state == ST_CONFIRM) {
      if (since(_confirmAt) >= CONFIRM_TIMEOUT_MS) { const char* w; logEvent("warn", "CONFIRM_TIMEOUT_DENIED"); confirm(false, w); }
      else if (since(_confirmAnn) >= CONFIRM_REPEAT_MS) announceConfirm();
      return;
    }
    const MissionStep& s = m->steps[i];
    uint32_t tmo = (uint32_t)s.timeoutS * 1000UL;
    if (_phase == PH_GATES) {
      const char* wait = nullptr;
      if ((s.flags & F_STEADY) && !systemSteady())                       wait = "STEADY";
      else if ((s.flags & F_ARM_CLEAR) && !armIsClear())                 wait = "ARM_CLEAR";
      else if ((s.flags & F_LIQUID) && policy == POL_FAST && liquidEvidence() != 1) wait = "LIQUID";
      if (wait) {
        waiting = wait;
        if (since(_stepT0) > tmo) {
          const char* o = !strcmp(wait, "STEADY") ? "TIMEOUT_NOT_STEADY" : !strcmp(wait, "ARM_CLEAR") ? "TIMEOUT_ARM_NOT_CLEAR" : "TIMEOUT_NO_LIQUID";
          endStep(o, FL_TIMEOUT | FL_SKIPPED);
        }
        return;
      }
      waiting = "";
      const char* why = nullptr;
      if (!startAction(s, why)) { endStep(why ? why : "START_FAILED", FL_FAILED); return; }
      _phase = PH_RUN;
      J.begin("step").kvu("ms", millis()).kvs("mission", m->name).kvu("i", i).kvs("act", ACTION_NAME[s.act])
       .kvs("bay", bayName(s.bay)).kvs("phase", "begin");
      emit();
      return;
    }
    const char* outcome = nullptr; uint16_t fl = 0;
    int8_t r = pollAction(s, outcome, fl);
    if (r != 0) { endStep(outcome ? outcome : (r > 0 ? "OK" : "FAILED"), (uint16_t)(fl | (r < 0 ? FL_FAILED : 0))); return; }
    if (tmo && since(_stepT0) > tmo) onTimeout(s);
  }

 private:
  enum Phase : uint8_t { PH_GATES, PH_RUN };
  bool clockLow() const { return since(_t0) > (m->budgetMs - m->clockLowMs); }
  bool battLow() const  { return ext.batt >= 0 && ext.batt < BATT_MIN_SECONDARY_PCT; }

  // A module switched off in config.h (or a bay not fitted) is not a fault: skip quietly, no flag.
  const char* disabledByConfig(const MissionStep& s) const {
    if (s.bay >= 0 && !BAYS[s.bay].exists) return "SKIPPED_BAY_NOT_FITTED";
    switch (s.act) {
      case A_TARE: case A_AWAIT_MASS: case A_WEIGH: return BAYS[s.bay].scale->enabled ? nullptr : "SKIPPED_DISABLED_SCALES";
      case A_LID_OPEN: case A_LID_CLOSE:            return BAYS[s.bay].lid->enabled() ? nullptr : "SKIPPED_NO_LID";
      case A_WHITE: case A_REFLECT:                 return ENABLE_REFLECT ? nullptr : "SKIPPED_DISABLED_REFLECT";
      case A_SWIR_WHITE: case A_SWIR:               return ENABLE_SWIR ? nullptr : "SKIPPED_DISABLED_SWIR";
      case A_MICRO_IMAGE:                           return ENABLE_MICROSCOPE ? nullptr : "SKIPPED_DISABLED_MICROSCOPE";
      case A_CARBONATE:                             return (ENABLE_CO2 && ENABLE_PUMPS) ? nullptr : "SKIPPED_DISABLED_CARBONATE";
      case A_PH_MEASURE:                            return ENABLE_PH ? nullptr : "SKIPPED_DISABLED_PH";
      case A_RINSE: case A_INTAKE:                  return ENABLE_PUMPS ? nullptr : "SKIPPED_DISABLED_PUMPS";
      default: return nullptr;
    }
  }
  // An ENABLED module that is dead right now IS a fault: skip and flag it.
  const char* hwMissing(const MissionStep& s) const {
    switch (s.act) {
      case A_TARE: case A_AWAIT_MASS: case A_WEIGH: return BAYS[s.bay].scale->healthy() ? nullptr : "SKIPPED_SCALE_OFFLINE";
      case A_WHITE: case A_REFLECT:  return refl.healthy() ? nullptr : "SKIPPED_REFLECT_OFFLINE";
      case A_SWIR_WHITE: case A_SWIR: return swir.healthy() ? nullptr : "SKIPPED_SWIR_OFFLINE";
      case A_CARBONATE:   return co2.healthy() ? nullptr : "SKIPPED_CO2_OFFLINE";
      case A_PH_MEASURE:  return phProbe.healthy() ? nullptr : "SKIPPED_PH_OFFLINE";
      default: return nullptr;
    }
  }
  void beginStep() {
    for (;;) {                                      // a loop, not recursion, when steps are skipped
      const MissionStep& s = m->steps[i];
      _stepT0 = millis(); _phase = PH_GATES; waiting = "";
      if (s.act == A_COMPLETE) { finishMission(); return; }
      const char* skip = nullptr; uint16_t fl = 0;
      if (const char* off = disabledByConfig(s))                                   { skip = off; fl = 0; }
      else if ((s.flags & F_SECONDARY) && clockLow())                              { skip = "SKIPPED_CLOCK_LOW"; fl = FL_SKIPPED; }
      else if ((s.flags & F_SECONDARY) && battLow())                               { skip = "SKIPPED_LOW_BATTERY"; fl = FL_SKIPPED; }
      else if ((s.flags & F_NEEDS_SAMPLE) && s.bay >= 0 && !BAYS[s.bay].massOk)    { skip = "SKIPPED_NO_SAMPLE"; fl = FL_SKIPPED | FL_NO_SAMPLE; }
      else if (const char* hw = hwMissing(s))                                      { skip = hw; fl = FL_SKIPPED | FL_HW; }
      if (skip) { reportEnd(s, skip, fl); if (!nextStep()) return; continue; }
      if ((s.flags & F_SECONDARY) && ext.batt < 0) flags |= FL_BATT_UNKNOWN;
      if ((s.flags & F_CONFIRM) && policy == POL_ASK && !_confirmed) {
        g_state = ST_CONFIRM; _confirmAt = millis(); confirmId++; announceConfirm();
        return;
      }
      g_state = ST_MISSION;
      return;                                       // gates + action are handled in service()
    }
  }
  bool nextStep() { i++; _confirmed = false; if (i >= m->n) { finishMission(); return false; } return true; }
  void endStep(const char* outcome, uint16_t fl) {
    reportEnd(m->steps[i], outcome, fl);
    if (nextStep()) beginStep();
  }
  void reportEnd(const MissionStep& s, const char* outcome, uint16_t fl) {
    flags |= fl;
    J.begin("step").kvu("ms", millis()).kvs("mission", m->name).kvu("i", i).kvs("act", ACTION_NAME[s.act])
     .kvs("bay", bayName(s.bay)).kvs("phase", "end").kvs("outcome", outcome)
     .kvu("dt_s", since(_stepT0) / 1000).kvu("step_fl", fl).kvu("flags", flags);   // step_fl = this step, flags = whole run so far
    emit();
  }
  void finishMission() {
    J.begin("event").kvu("ms", millis()).kvs("lvl", "info").kvs("code", "MISSION_COMPLETE").kvs("mission", m->name)
     .kvu("elapsed_s", since(_t0) / 1000).kvu("flags", flags).kvs("policy", POLICY_NAME[policy]);
    emit();
    m = nullptr; waiting = "";
    if (g_state != ST_SAFE) g_state = ST_IDLE;
  }
  void announceConfirm() {
    const MissionStep& s = m->steps[i];
    const char* reason = s.act == A_CARBONATE ? "HCL_ACID_DOSE" : s.act == A_PH_MEASURE ? "IS_THE_PH_PROBE_IN_THE_LIQUID"
                       : s.act == A_INTAKE ? "IS_THE_INTAKE_TUBE_IN_THE_LIQUID" : "OPERATOR_CHECK";
    J.begin("confirm_request").kvu("ms", millis()).kvu("id", confirmId).kvs("mission", m->name).kvu("step", i)
     .kvs("action", ACTION_NAME[s.act]).kvs("bay", bayName(s.bay)).kvs("reason", reason)
     .kv("batt", ext.batt).kvu("timeout_s", (CONFIRM_TIMEOUT_MS - min(since(_confirmAt), CONFIRM_TIMEOUT_MS)) / 1000);
    emit();
    _confirmAnn = millis();
  }

  // ---- actions ----
  bool startAction(const MissionStep& s, const char*& why) {
    _scaleSeq0 = (s.bay >= 0) ? BAYS[s.bay].scale->seq() : 0;
    switch (s.act) {
      case A_PREFLIGHT:  return true;
      case A_TARE:       if (!BAYS[s.bay].scale->startTare()) { why = "SCALE_OFFLINE"; return false; } return true;
      case A_LID_OPEN:   return BAYS[s.bay].lid->open(why);
      case A_LID_CLOSE:  return BAYS[s.bay].lid->close(why);
      case A_AWAIT_MASS: BAYS[s.bay].massOk = false; return true;
      case A_WEIGH:      return true;
      case A_WHITE:      return startWhite(s.bay, why);
      case A_REFLECT:    { char tag[24]; snprintf(tag, sizeof tag, "%s_%s", m->name, BAYS[s.bay].name); return startReflect(s.bay, tag, why); }
      case A_SWIR_WHITE: return startSwir(s.bay, true, "", why);
      case A_SWIR:       { char tag[24]; snprintf(tag, sizeof tag, "%s_%s", m->name, BAYS[s.bay].name); return startSwir(s.bay, false, tag, why); }
      case A_MICRO_IMAGE:
        if (!microLight.on(0, why)) return false;
        imageId++;
        J.begin("request").kvu("ms", millis()).kvs("what", "micro_image").kvu("id", imageId).kvs("bay", BAYS[s.bay].name);
        emit(); return true;
      case A_CARBONATE:  return carb.start((uint8_t)s.bay, why);
      case A_PH_MEASURE: return true;
      case A_RINSE:      return dose.start(&pumpRinse, s.param, false, 0, why);
      case A_INTAKE: {
        float ml = fminf(s.param, ASTRO_MAX_SAMPLE_ML - astroMl);
        if (ml <= 0.5f) { why = "SAMPLE_BOTTLE_AT_CAP"; return false; }
        return dose.start(&pumpSample, ml, true, 0, why);
      }
      default: return true;
    }
  }
  // 0 = running, 1 = done OK, -1 = failed
  int8_t pollAction(const MissionStep& s, const char*& outcome, uint16_t& fl) {
    switch (s.act) {
      case A_PREFLIGHT: return preflight(outcome, fl);
      case A_TARE:
        if (BAYS[s.bay].scale->jobBusy()) return 0;
        if (BAYS[s.bay].scale->jobOk()) return 1;
        outcome = "TARE_UNSTABLE"; return -1;
      case A_LID_OPEN: case A_LID_CLOSE: {
        StepperLid* l = BAYS[s.bay].lid;
        if (l->moving()) return 0;
        bool want = s.act == A_LID_OPEN;
        if (l->state() == (want ? StepperLid::OPEN : StepperLid::CLOSED)) return 1;
        outcome = l->fault() ? l->fault() : "LID_NOT_IN_POSITION"; return -1;
      }
      case A_AWAIT_MASS:
        if (BAYS[s.bay].scale->enough(s.param)) { BAYS[s.bay].massOk = true; return 1; }
        return 0;
      case A_WEIGH: {
        Hx711Scale* sc = BAYS[s.bay].scale;
        if (!systemSteady() || !sc->stable() || sc->seq() - _scaleSeq0 < SCALE_WINDOW) return 0;   // fresh, still, settled
        float g = sc->grams(); Bay& b = BAYS[s.bay];
        b.lastG = g; b.massOk = !isnan(g) && g >= SAMPLE_MIN_G;
        bool enough = !isnan(g) && g >= s.param;
        J.begin("result").kvs("kind", "weigh").kvu("ms", millis()).kvs("bay", b.name).kvf("grams", g, 1)
         .kvf("target_g", s.param, 0).kvb("enough", enough).kvb("stable", true).kvb("tilt_comp", sc->tiltCompensated())
         .kvf("pitch", imu.pitch(), 1).kvf("roll", imu.roll(), 1).kvb("calibrated", sc->calibrated());
        if (s.bay == DEEP) J.kv("depth_mm", ext.depthMm);
        emit();
        if (!sc->calibrated()) fl |= FL_UNCAL;
        if (!enough) { fl |= FL_INSUFFICIENT; outcome = b.massOk ? "BELOW_TARGET" : "EMPTY"; }
        return 1;
      }
      case A_WHITE: case A_REFLECT:
        if (refl.busy()) return 0;
        if (refl.lastOk()) return 1;
        outcome = refl.lastError(); return -1;
      case A_SWIR_WHITE: case A_SWIR:
        if (swir.busy()) return 0;
        if (swir.lastOk()) return 1;
        outcome = swir.lastError(); return -1;
      case A_MICRO_IMAGE:
        if (imageAck != imageId) return 0;
        microLight.off(); return 1;
      case A_CARBONATE:
        if (carb.busy()) return 0;
        if (carb.ok()) return 1;
        outcome = carb.err(); return -1;
      case A_PH_MEASURE:
        if (!phProbe.stable()) return 0;
        emitPh("mission");
        if (!phProbe.calibrated()) { fl |= FL_UNCAL; outcome = "OK_BUT_UNCALIBRATED"; }
        return 1;
      case A_RINSE: case A_INTAKE:                   // dose.poll() runs every loop pass (main loop)
        if (dose.busy()) return 0;
        if (!dose.ok()) { outcome = dose.err(); return -1; }
        if (s.act == A_INTAKE) {
          J.begin("result").kvs("kind", "intake").kvu("ms", millis()).kvf("ml", dose.ml(), 1)
           .kvf("bottle_ml", astroMl, 1).kvf("cap_ml", ASTRO_MAX_SAMPLE_ML, 0).kvb("pump_cal", pumpSample.calibrated());
          emit();
        }
        return 1;
      default: return 1;
    }
  }
  // Preflight looks at what THIS mission's steps will use, checks each, and
  // reports. It never blocks: a FAST run with warnings falls back to ASK.
  int8_t preflight(const char*& outcome, uint16_t& fl) {
    J.begin("result").kvs("kind", "preflight").kvu("ms", millis()).kvs("mission", m->name).obj("checks");
    bool warn = false;
    auto chk = [&](const char* k, bool ok) { J.kvb(k, ok); if (!ok) warn = true; };
    bool seen[16] = { false }; bool bays[N_BAYS] = { false }; bool lids[N_BAYS] = { false };
    for (uint8_t k = 0; k < m->n; k++) {
      const MissionStep& s = m->steps[k]; seen[s.act] = true;
      if (s.bay >= 0 && BAYS[s.bay].exists) { if (s.act == A_TARE || s.act == A_WEIGH || s.act == A_AWAIT_MASS) bays[s.bay] = true;
                                               if ((s.act == A_LID_OPEN || s.act == A_LID_CLOSE) && BAYS[s.bay].lid->enabled()) lids[s.bay] = true; }
    }
    char key[20];
    for (uint8_t b = 0; b < N_BAYS; b++) if (bays[b] && ENABLE_SCALES) {
      snprintf(key, sizeof key, "%s_scale", BAYS[b].scale->name); chk(key, BAYS[b].scale->healthy());
      snprintf(key, sizeof key, "%s_cal", BAYS[b].scale->name);   chk(key, BAYS[b].scale->calibrated());
    }
    for (uint8_t b = 0; b < N_BAYS; b++) if (lids[b]) { snprintf(key, sizeof key, "%s_ok", BAYS[b].lid->name); chk(key, !BAYS[b].lid->fault()); }
    if (seen[A_WEIGH] && imu.enabled) chk("imu", imu.healthy());
    if ((seen[A_WHITE] || seen[A_REFLECT]) && ENABLE_REFLECT) chk("refl", refl.healthy());
    if ((seen[A_SWIR] || seen[A_SWIR_WHITE]) && ENABLE_SWIR)  chk("swir", swir.healthy());
    if (seen[A_CARBONATE] && ENABLE_CO2 && ENABLE_PUMPS) { chk("co2", co2.healthy()); chk("acid_cal", pumpAcid.calibrated()); chk("cup_fresh", !BAYS[ANALYSIS_BAY].spent); }
    if (seen[A_PH_MEASURE] && ENABLE_PH) { chk("ph", phProbe.healthy()); chk("ph_cal", phProbe.calibrated());
                                           if (ENABLE_PH_TEMP) chk("ph_liquid_temp", !isnan(liqTemp.tempC())); }
    if (seen[A_INTAKE] && ENABLE_PUMPS) chk("sample_pump_cal", pumpSample.calibrated());
    if (seen[A_RINSE] && ENABLE_PUMPS)  chk("rinse_pump_cal", pumpRinse.calibrated());
    if ((seen[A_PH_MEASURE] || seen[A_INTAKE]) && policy == POL_FAST) chk("liquid_evidence", liquidSourceKnown());   // FAST needs an "in liquid" sensor
    chk("estop_released", !Safety::estop);
    if (g_mode == MODE_ROS2) { chk("bridge", g_hostLinkUp); chk("rover_motion_fresh", ext.moving.fresh(EXT_FRESH_MS)); }
    J.close('}').kvb("ok", !warn).kvs("policy", POLICY_NAME[policy]);
    emit();
    if (warn) {
      fl |= FL_UNCAL; outcome = "WARN_SEE_PREFLIGHT";
      if (policy == POL_FAST) { policy = POL_ASK; fl |= FL_FALLBACK_ASK; logEvent("warn", "POLICY_FALLBACK_ASK", "preflight warnings: this run asks before acid and liquid steps"); }
    }
    return 1;                                        // preflight informs, it never blocks
  }
  void onTimeout(const MissionStep& s) {
    if (s.act == A_AWAIT_MASS) {                     // a partial sample is still a sample
      float g = BAYS[s.bay].scale->grams();
      bool some = !isnan(g) && g >= SAMPLE_MIN_G;
      BAYS[s.bay].massOk = some;
      endStep(some ? "TIMEOUT_PARTIAL_SAMPLE" : "TIMEOUT_NO_SAMPLE", FL_TIMEOUT | (some ? FL_INSUFFICIENT : FL_NO_SAMPLE));
      return;
    }
    stopAction(s);
    endStep("TIMEOUT", FL_TIMEOUT);
  }
  void stopAction(const MissionStep& s) {
    switch (s.act) {
      case A_LID_OPEN: case A_LID_CLOSE: BAYS[s.bay].lid->stop(); break;
      case A_MICRO_IMAGE: microLight.off(); break;
      case A_WHITE: case A_REFLECT: refl.abort("MISSION_STOP"); break;
      case A_SWIR_WHITE: case A_SWIR: swir.abort("MISSION_STOP"); break;
      case A_CARBONATE: carb.abort("MISSION_STOP"); break;
      case A_RINSE: case A_INTAKE: dose.abort("MISSION_STOP"); break;
      case A_TARE: BAYS[s.bay].scale->safe(); break;
      default: break;
    }
  }
 public:
  static void emitPh(const char* label) {
    J.begin("result").kvs("kind", "ph").kvu("ms", millis()).kvs("label", label).kvf("ph", phProbe.ph(), 2)
     .kvf("volts", phProbe.volts(), 4).kvf("temp_c", phProbe.tempC(), 2).kvs("temp_src", phProbe.tempSrc())
     .kvb("stable", phProbe.stable()).kvb("calibrated", phProbe.calibrated()).kvf("slope_mv_ph", phProbe.slopeMvPerPh(), 1);
    emit();
  }
 private:
  Phase _phase = PH_GATES; bool _confirmed = false;
  uint32_t _t0 = 0, _armAt = 0, _stepT0 = 0, _confirmAt = 0, _confirmAnn = 0, _scaleSeq0 = 0;
};
Sequencer seq;

// ============================================================================
// SAFE: reachable from anywhere, LATCHED until CMD:CLEAR_SAFE. The state
// flips FIRST, so every actuator refuses before we stop them one by one.
// ============================================================================
void enterSafe(const char* why) {
  bool already = g_state == ST_SAFE;
  g_state = ST_SAFE;
  illum.off(); refl.abort("SAFE"); swir.abort("SAFE"); selftest.abort();
  for (Pump* p : PUMPS) p->stop();
  for (StepperLid* l : LIDS) l->stop();
  purge.off(); microLight.off(); autonLight.off();
  carb.abort("SAFE"); dose.abort("SAFE");
  seq.abort(why);
  if (!already) { J.begin("safe").kvu("ms", millis()).kvs("why", why); emit(); }
}
bool clearSafe(const char*& why) {
  if (g_state != ST_SAFE) { why = "NOT_IN_SAFE"; return false; }
  if (Safety::estop)      { why = "RELEASE_THE_ESTOP_BUTTON_FIRST"; return false; }
  g_state = ST_IDLE; return true;
}
bool anythingMoving() {
  if (refl.busy() || swir.busy() || carb.busy() || dose.busy() || selftest.busy() || purge.isOn()) return true;
  for (Pump* p : PUMPS) if (p->busy()) return true;
  for (StepperLid* l : LIDS) if (l->moving()) return true;
  return false;
}
