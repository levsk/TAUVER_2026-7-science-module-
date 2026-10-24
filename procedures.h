#pragma once
// ============================================================================
//  procedures.h — multi-step operations built from sensors + actuators.
//    hardware       the lids, pumps and outputs (one object each)
//    Bays           a TABLE: rock / regolith / deep = a scale + a lid + bookkeeping
//    analysis bay   the regolith bay: reflectance probe + CO2 sensor in its lid
//    DoseProc       pump a volume forward, optionally pull the line back
//    CarbonateProc  baseline -> acid dose -> watch CO2 -> purge -> verdict
//    SelfTest       one plausibility check per enabled module, plus a lamp loop
//  Every procedure: start() refuses with a reason, poll() never blocks,
//  abort() leaves every actuator OFF.
//
//  Fluid domains are separate on purpose (Planetary Protection + judges check
//  cross-contamination): GENERAL module = acid line only; ASTROBIO module =
//  liquid-sample + rinse lines only. No pump ever serves both.
// ============================================================================
#include "sensors.h"
#include "reflect.h"
#include "swir.h"

bool reflectAllowed() { return Safety::actuationAllowed(); }     // reflect.h asks the sketch: may lamps burn now?

// ---------------------------------------------------------------- hardware ---
StepperLid lidRock("lid_rock", 0, PIN_LID_ROCK, PIN_LID_END_ROCK, ENABLE_LIDS);
StepperLid lidReg ("lid_reg",  1, PIN_LID_REG,  PIN_LID_END_REG,  ENABLE_LIDS);
StepperLid lidDeep("lid_deep", 2, PIN_LID_DEEP, PIN_LID_END_DEEP, ENABLE_LIDS && ENABLE_DEEP_LID);
StepperLid* const LIDS[N_BAYS] = { &lidRock, &lidReg, &lidDeep };

Pump pumpAcid  ("acid",   "general",  0, PIN_ACID_IN1,   PIN_ACID_IN2);     // HCl, carbonate test
Pump pumpSample("sample", "astrobio", 1, PIN_SAMPLE_IN1, PIN_SAMPLE_IN2);   // liquid sample intake
Pump pumpRinse ("rinse",  "astrobio", 2, PIN_RINSE_IN1,  PIN_RINSE_IN2);    // DI-water rinse
Pump* const PUMPS[3] = { &pumpAcid, &pumpSample, &pumpRinse };

Output purge     ("purge",       PIN_PURGE,       ENABLE_PURGE);
Output microLight("micro_light", PIN_MICRO_LIGHT, ENABLE_MICROSCOPE);
Output autonLight("auton_light", PIN_AUTON_LIGHT, PIN_AUTON_LIGHT != NO_PIN);

Pump* findPump(const char* n) { for (Pump* p : PUMPS) if (!strcasecmp(n, p->name)) return p; return nullptr; }

// -------------------------------------------------------------------- bays ---
struct Bay {
  const char* name; Hx711Scale* scale; StepperLid* lid; bool exists;
  bool massOk; bool spent; float lastG;
};
Bay BAYS[N_BAYS] = {
  { "ROCK",     &scaleRock, &lidRock, true,            false, false, NAN },
  { "REGOLITH", &scaleReg,  &lidReg,  true,            false, false, NAN },
  { "DEEP",     &scaleDeep, &lidDeep, ENABLE_DEEP_BAY, false, false, NAN },
};
constexpr int8_t ROCK = 0, REG = 1, DEEP = 2, NOBAY = -1;
// "ROCK"|"R"|"0"  /  "REGOLITH"|"REG"|"1"  /  "DEEP"|"D"|"2". Returns -1 if unknown (or not fitted).
int8_t bayIndex(const char* s) {
  if (!s || !*s) return -1;
  int8_t b = -1;
  if (!strcasecmp(s, "ROCK") || !strcasecmp(s, "R") || !strcmp(s, "0")) b = ROCK;
  else if (!strcasecmp(s, "REGOLITH") || !strcasecmp(s, "REG") || !strcmp(s, "1")) b = REG;
  else if (!strcasecmp(s, "DEEP") || !strcasecmp(s, "D") || !strcmp(s, "2")) b = DEEP;
  return (b >= 0 && BAYS[b].exists) ? b : -1;
}
float astroMl = 0;     // liquid already drawn into the Astro-Bio sample bottle (ERC: < 100 mL)

// ------------------------------------------------------------ analysis bay ---
// The reflectance probe looks down from the analysis bay's lid. The cup floor
// is a PTFE disc: the WHITE is taken while the cup is EMPTY, the SCAN after
// the sample is in. Both need the lid closed (the probe is IN the lid, and the
// chamber must be dark). Acid ruins the cup: after a carbonate test, BAY_RESET.
bool reflectGate(int8_t bay, bool white, const char*& why) {
  if (bay != (int8_t)ANALYSIS_BAY)            { why = "NO_PROBE_IN_THIS_BAY"; return false; }
  const Bay& b = BAYS[bay];
  if (!b.lid->isClosed())                     { why = "LID_NOT_CLOSED"; return false; }
  if (b.spent)                                { why = "CUP_SPENT_SEND_BAY_RESET"; return false; }
  if (white) {
    float g = b.scale->grams();
    if (!isnan(g) && g > WHITE_MAX_CUP_G)     { why = "CUP_NOT_EMPTY_WHITE_NEEDS_THE_BARE_PTFE"; return false; }
  }
  return true;
}
bool startWhite(int8_t bay, const char*& why)                  { return reflectGate(bay, true, why) && refl.startWhite(why); }
bool startReflect(int8_t bay, const char* tag, const char*& why) { return reflectGate(bay, false, why) && refl.startScan(tag, why); }
bool startSwir(int8_t bay, bool white, const char* tag, const char*& why) {
  if (!reflectGate(bay, white, why)) return false;
  return white ? swir.startWhite(why) : swir.startScan(tag, why);
}

// ---------------------------------------------------------------- DoseProc ---
// The Astro-Bio bottle counts what was ACTUALLY pumped, when the pump stops
// (2.0 counted a manual dose before pumping it).
class DoseProc {
 public:
  bool start(Pump* p, float ml, bool retract, uint32_t fallbackMs, const char*& why) {
    if (busy()) { why = "DOSE_RUNNING"; return false; }
    _p = p; _retract = retract; _ok = false; _err = nullptr; _ml = 0;
    bool started = p->calibrated() ? p->doseMl(ml, why)
                 : (fallbackMs ? p->run(Pump::FWD, PUMP_DEFAULT_DUTY, fallbackMs, why) : (why = "PUMP_UNCALIBRATED", false));
    if (!started) return false;
    _st = D_FWD; return true;
  }
  void poll() {
    if (_st == D_IDLE || _st == D_DONE) return;
    if (!Safety::actuationAllowed()) { abort("SAFE"); return; }
    if (_p->busy()) return;
    const char* why = nullptr;
    if (_st == D_FWD) {
      account();
      if (_retract) {
        if (_p->run(Pump::REV, 255, PUMP_RETRACT_MS, why)) { _st = D_REV; return; }
        if (!strcmp(why, "NO_REVERSE_WIRED")) { finish(true, nullptr); return; }   // nothing to retract with
        finish(false, why); return;
      }
    }
    finish(true, nullptr);
  }
  void abort(const char* why) {
    if (!_p) return;
    bool wasFwd = _st == D_FWD;
    _p->stop();
    if (wasFwd) account();                           // a stopped dose still moved liquid: count it
    if (busy()) finish(false, why);
  }
  bool busy() const { return _st == D_FWD || _st == D_REV; }
  bool ok() const   { return _ok; }
  float ml() const  { return _ml; }                  // volume moved forward by the last dose
  const Pump* pump() const { return _p; }
  const char* err() const { return _err ? _err : ""; }
 private:
  void account() { _ml = _p->lastMl(); if (_p == &pumpSample) astroMl += _ml; }
  void finish(bool ok, const char* err) { _ok = ok; _err = err; _st = D_DONE; }
  enum St : uint8_t { D_IDLE, D_FWD, D_REV, D_DONE };
  St _st = D_IDLE; Pump* _p = nullptr; bool _retract = false, _ok = false; float _ml = 0; const char* _err = nullptr;
};
DoseProc dose;      // one fluid operation at a time, whichever module

// ----------------------------------------------------------- CarbonateProc ---
// Detection only, by physics: 1 mg CaCO3 releases ~0.24 mL CO2 = ~2400 ppm in a
// 100 mL headspace, and the SCD41 is specified to 5000 ppm. So the honest
// outputs are PRESENT / ABSENT, the rise and its fastest rate. Runs LAST on a
// sample (destructive), in the sealed analysis chamber (lid closed).
class CarbonateProc {
 public:
  bool start(uint8_t bay, const char*& why) {
    if (busy())                         { why = "CARB_RUNNING"; return false; }
    if (!Safety::actuationAllowed())    { why = Safety::whyNot(); return false; }
    if (bay != ANALYSIS_BAY)            { why = "NO_CO2_SENSOR_IN_THIS_BAY"; return false; }
    if (!co2.healthy() || !co2.seq())   { why = "CO2_NOT_READY"; return false; }
    if (BAYS[bay].spent)                { why = "CUP_SPENT_SEND_BAY_RESET"; return false; }
    if (!BAYS[bay].lid->isClosed())     { why = "LID_NOT_CLOSED_CHAMBER_NOT_SEALED"; return false; }
    if (dose.busy() || pumpAcid.busy()) { why = "PUMP_BUSY"; return false; }
    _bay = bay; _ok = false; _err = nullptr; _dosed = false;
    _n = 0; _sum = 0; _min = INT32_MAX; _max = INT32_MIN; _peak = 0; _tPeakS = 0; _rate = 0; _prevV = -1;
    _seq = co2.seq(); _t0 = millis(); _st = C_BASELINE;
    logEvent("info", "CARB_BASELINE", "30 s flat-CO2 leak check");
    return true;
  }
  void poll() {
    if (_st == C_IDLE || _st == C_DONE) return;
    if (!Safety::actuationAllowed()) { abort("SAFE"); return; }
    if (_st != C_PURGE && !co2.healthy()) { abort("CO2_LOST"); return; }
    bool fresh = co2.seq() != _seq; int32_t v = co2.co2();
    if (fresh) _seq = co2.seq();
    const char* why = nullptr;
    switch (_st) {
      case C_BASELINE:
        if (fresh) { _n++; _sum += v; _min = min(_min, v); _max = max(_max, v); }
        if (since(_t0) < CARB_BASELINE_MS) return;
        if (_n < 3)                              { abort("BASELINE_TOO_FEW_SAMPLES"); return; }
        _base = (int32_t)(_sum / _n);
        if (_max - _min > CARB_DRIFT_MAX_PPM)    { abort("BASELINE_DRIFT_CHAMBER_LEAKS"); return; }
        if (_base > CARB_BASELINE_MAX_PPM)       { abort("CHAMBER_NOT_PURGED"); return; }
        if (!dose.start(&pumpAcid, CARB_DOSE_ML, true, CARB_DOSE_MS_UNCAL, why)) { abort(why); return; }
        _dosed = true; _st = C_DOSE; _peak = _base; logEvent("info", "CARB_ACID_DOSED");
        return;
      case C_DOSE:                                   // dose.poll() runs every loop pass (main loop)
        if (dose.busy()) return;
        if (!dose.ok()) { abort(dose.err()); return; }
        _st = C_REACT; _t0 = millis(); _prevV = _base; _prevT = millis();
        return;
      case C_REACT:
        if (fresh) {
          if (v > _peak) { _peak = v; _tPeakS = since(_t0) / 1000; }
          float dt = since(_prevT) / 1000.0f;
          if (dt > 0.5f) { float r = (v - _prevV) / dt; if (r > _rate) _rate = r; }
          _prevV = v; _prevT = millis();
        }
        if (since(_t0) < CARB_REACT_MS) return;
        report();
        if (ENABLE_PURGE && purge.on(CARB_PURGE_MS, why)) { _st = C_PURGE; _t0 = millis(); return; }
        _st = C_DONE; return;
      case C_PURGE:
        if (purge.isOn() && since(_t0) < CARB_PURGE_MS + 100) return;
        purge.off(); _st = C_DONE; return;
      default: return;
    }
  }
  void abort(const char* why) {
    dose.abort(why); pumpAcid.stop(); purge.off();
    if (_dosed) BAYS[_bay].spent = true;           // acid touched it: never reuse this cup
    if (busy()) { _err = why; logEvent("warn", "CARB_ABORTED", why); }
    _st = C_DONE; _ok = false;
  }
  bool busy() const { return _st != C_IDLE && _st != C_DONE; }
  bool ok() const   { return _ok; }
  const char* err() const { return _err ? _err : ""; }
  const char* phase() const { static const char* const N[] = { "IDLE", "BASELINE", "DOSE", "REACT", "PURGE", "DONE" }; return N[_st]; }
 private:
  void report() {
    int32_t delta = _peak - _base;                 // int32 on purpose: an unsigned wrap once faked a reaction
    bool detected = delta >= CARB_DETECT_PPM, over = _peak >= CO2_OVER_RANGE_PPM;
    BAYS[_bay].spent = true; _ok = true;
    J.begin("result").kvs("kind", "carbonate").kvu("ms", millis()).kvs("bay", BAYS[_bay].name)
     .kv("baseline_ppm", _base).kv("baseline_drift_ppm", _max - _min).kv("peak_ppm", _peak).kv("delta_ppm", delta)
     .kvu("t_peak_s", _tPeakS).kvf("rise_ppm_s", _rate, 1).kvb("detected", detected).kvb("over_range", over)
     .kvs("verdict", !detected ? "ABSENT" : (over ? "PRESENT_OVER_RANGE" : "PRESENT"))
     .kvs("quant", "detection_only").kvb("acid_pump_cal", pumpAcid.calibrated()).kvf("acid_ml", dose.ml(), 2);
    emit();
  }
  enum St : uint8_t { C_IDLE, C_BASELINE, C_DOSE, C_REACT, C_PURGE, C_DONE };
  St _st = C_IDLE; uint8_t _bay = 0; bool _ok = false, _dosed = false; const char* _err = nullptr;
  uint32_t _t0 = 0, _seq = 0, _tPeakS = 0, _prevT = 0; uint16_t _n = 0; int64_t _sum = 0;
  int32_t _min = 0, _max = 0, _base = 0, _peak = 0, _prevV = 0; float _rate = 0;
};
CarbonateProc carb;

// --------------------------------------------------------------- SelfTest ---
// CMD:SELFTEST : one frame that says, module by module, "is this plausible?"
//   passive checks for every enabled module (values in range, calibrated)
//   + the LAMP LOOP: dark, VIS, NIR frames at a fixed exposure. Each group must
//     raise ITS OWN bands. Catches a dead MOSFET, a missing LED, swapped gates.
// CMD:SELFTEST=MOTION also jogs every fitted lid a little and back (MANUAL only).
// It is internal QA, not a rules item.
// ----------------------------------------------------------------------------
class SelfTest {
 public:
  bool start(bool motion, const char*& why) {
    if (busy())                       { why = "SELFTEST_RUNNING"; return false; }
    if (motion && !armIsClear())      { why = "ARM_NOT_CLEAR"; return false; }
    _motion = motion; _st = T_LAMP_START; _lidI = 0; _t0 = millis(); _lampMsg = "SKIPPED"; _lampOk = true; _lampRan = false;
    for (uint8_t i = 0; i < N_BAYS; i++) { _lidMsg[i] = "SKIPPED"; _lidOk[i] = true; }
    return true;
  }
  bool busy() const { return _st != T_IDLE; }
  void abort() { if (!busy()) return; illum.off(); for (StepperLid* l : LIDS) l->stop(); _st = T_IDLE; }
  void poll() {
    if (_st == T_IDLE) return;
    if (!Safety::actuationAllowed() && _st != T_REPORT) { illum.off(); _lampMsg = _lampRan ? "INTERRUPTED" : _lampMsg; _st = T_REPORT; }
    if (since(_t0) > 30000 && _st != T_REPORT) { illum.off(); _st = T_REPORT; }
    switch (_st) {
      case T_LAMP_START: {
        SpecDrv* d = refl.drv();
        if (!ENABLE_REFLECT || !refl.healthy() || !d || refl.busy()) { _lampMsg = ENABLE_REFLECT ? "SENSOR_NOT_READY" : "DISABLED"; _lampOk = !ENABLE_REFLECT; _st = T_LIDS; return; }
        _lampRan = true; _frame = 0; startFrame(); _st = T_LAMP_RUN; return;
      }
      case T_LAMP_RUN: {
        SpecDrv* d = refl.drv();
        if (!d || !refl.healthy()) { illum.off(); _lampMsg = "SENSOR_LOST"; _lampOk = false; _st = T_LIDS; return; }
        if (since(_ts) < REFLECT_SETTLE_MS) return;
        if (!_started) { d->start(d->gainStart(), d->tintQuant(REFLECT_TINT_START_MS), 0); _started = true; return; }
        int8_t r = d->poll();
        if (r == 0) return;
        illum.off();
        if (r < 0) { _lampMsg = "SENSOR_IO_ERROR"; _lampOk = false; _st = T_LIDS; return; }
        for (uint8_t b = 0; b < d->nb; b++) _raw[_frame][b] = d->raw[b];
        if (++_frame < 3) { startFrame(); return; }
        judgeLamps(d);
        _st = T_LIDS; return;
      }
      case T_LIDS: {
        if (!_motion) { _st = T_REPORT; return; }
        while (_lidI < N_BAYS && !LIDS[_lidI]->enabled()) _lidI++;
        if (_lidI >= N_BAYS) { _st = T_REPORT; return; }
        StepperLid* l = LIDS[_lidI];
        if (l->moving()) return;
        const char* why = nullptr;
        if (_lidPhase == 0) { _lidPos0 = l->pos(); if (!l->jog(+LID_JOG_STEPS, why)) { _lidMsg[_lidI] = why; _lidOk[_lidI] = false; nextLid(); } else _lidPhase = 1; return; }
        if (_lidPhase == 1) { if (l->fault()) { _lidMsg[_lidI] = l->fault(); _lidOk[_lidI] = false; nextLid(); return; }
                              if (!l->jog(_lidPos0 - l->pos(), why)) { _lidMsg[_lidI] = why; _lidOk[_lidI] = false; nextLid(); } else _lidPhase = 2; return; }
        _lidOk[_lidI] = !l->fault() && l->pos() == _lidPos0; _lidMsg[_lidI] = _lidOk[_lidI] ? "JOGGED_AND_BACK" : (l->fault() ? l->fault() : "NOT_BACK");
        nextLid(); return;
      }
      case T_REPORT: report(); _st = T_IDLE; return;
      default: return;
    }
  }
 private:
  enum St : uint8_t { T_IDLE, T_LAMP_START, T_LAMP_RUN, T_LIDS, T_REPORT };
  void startFrame() {                               // frame 0 = dark, 1 = VIS group, 2 = NIR group
    const char* why;
    if (_frame == 0) illum.off(); else illum.on((uint8_t)(1 << (_frame - 1)), 3000, why);
    _ts = millis(); _started = false;
  }
  void nextLid() { _lidI++; _lidPhase = 0; }
  void judgeLamps(SpecDrv* d) {
    float visV = 0, visN = 0, nirV = 0, nirN = 0;   // best rise: [group frame][band class]
    bool sat = false;
    for (uint8_t b = 0; b < d->nb; b++) {
      float dv = (float)_raw[1][b] - _raw[0][b], dn = (float)_raw[2][b] - _raw[0][b];
      if (_raw[1][b] >= 65000 || _raw[2][b] >= 65000) sat = true;
      if (d->wl[b] < 700)      { visV = fmaxf(visV, dv); nirV = fmaxf(nirV, dn); }
      else if (d->wl[b] > 750) { visN = fmaxf(visN, dv); nirN = fmaxf(nirN, dn); }
    }
    _visRise = visV; _nirRise = nirN;
    bool visOk = visV >= LAMPTEST_MIN_COUNTS, nirOk = nirN >= LAMPTEST_MIN_COUNTS;
    bool swapped = visV < nirV && nirN < visN && (nirV >= LAMPTEST_MIN_COUNTS || visN >= LAMPTEST_MIN_COUNTS);
    _lampOk = visOk && nirOk && !swapped;
    _lampMsg = swapped ? "GATES_SWAPPED" : (!visOk && !nirOk) ? "BOTH_GROUPS_DARK" : !visOk ? "VIS_GROUP_DARK" : !nirOk ? "NIR_GROUP_DARK"
             : sat ? "OK_BUT_SATURATED" : "OK";
  }
  void chk(const char* k, bool en, bool ok, const char* msg, bool& all) {
    if (!en) return;
    J.obj(k).kvb("ok", ok).kvs("msg", msg).close('}');
    if (!ok) all = false;
  }
  void report() {
    bool all = true;
    J.begin("result").kvs("kind", "selftest").kvu("ms", millis()).kvs("profile", PROFILE_NAME).obj("checks");
    chk("estop", ENABLE_ESTOP_IN, !Safety::estop, Safety::estop ? "PRESSED" : "RELEASED", all);
    chk("imu", imu.enabled, imu.healthy() && fabsf(imu.gMag() - 1.0f) < 0.1f && imu.biasOk(),
        !imu.healthy() ? "OFFLINE" : !imu.biasOk() ? "GYRO_BIAS_NOT_SET_HOLD_STILL" : fabsf(imu.gMag() - 1.0f) >= 0.1f ? "GRAVITY_NOT_1G" : "OK", all);
    for (uint8_t i = 0; i < N_BAYS; i++) {
      if (!BAYS[i].exists) continue;
      Hx711Scale* s = BAYS[i].scale; float g = s->grams();
      char k[16]; snprintf(k, sizeof k, "scale_%s", s->name);
      chk(k, s->enabled, s->healthy() && s->calibrated() && !isnan(g) && g > -5.0f && g < 3000.0f,
          !s->healthy() ? "OFFLINE" : !s->calibrated() ? "UNCALIBRATED" : (g <= -5.0f || g >= 3000.0f) ? "OUT_OF_RANGE_TARE_AGAIN" : "OK", all);
    }
    float t = met.tempC(), p = met.hPa();
    chk("met", met.enabled, met.healthy() && t > -20 && t < 60 && p > 700 && p < 1100, !met.healthy() ? "OFFLINE" : (t <= -20 || t >= 60 || p <= 700 || p >= 1100) ? "IMPLAUSIBLE" : "OK", all);
    int32_t c = co2.co2();
    chk("co2", co2.enabled, co2.healthy() && c >= 300 && c <= 5000, !co2.healthy() ? "OFFLINE" : c < 0 ? "NO_DATA_YET" : (c < 300 || c > 5000) ? "IMPLAUSIBLE" : "OK", all);
    chk("wind", wind.enabled, wind.healthy(), wind.healthy() ? "OK" : "NO_REPLY", all);
    float lt = liqTemp.tempC();
    chk("ph_temp", liqTemp.enabled, liqTemp.healthy() && !isnan(lt), liqTemp.healthy() ? (isnan(lt) ? "NO_DATA_YET" : "OK") : "OFFLINE", all);
    chk("ph", phProbe.enabled, phProbe.healthy() && phProbe.calibrated(), !phProbe.healthy() ? "OFFLINE" : !phProbe.calibrated() ? "UNCALIBRATED" : "OK", all);
    chk("liquid", liquid.enabled, liquid.healthy(), liquid.healthy() ? "OK" : "OFFLINE", all);
    chk("refl", refl.enabled, refl.healthy() && refl.whiteValid(), !refl.healthy() ? "OFFLINE" : !refl.whiteValid() ? "NO_WHITE_YET" : "OK", all);
    chk("lamps", ENABLE_REFLECT, _lampOk, _lampMsg, all);
    chk("swir", swir.enabled, swir.healthy(), swir.healthy() ? "OK" : "OFFLINE", all);
    for (uint8_t i = 0; i < N_BAYS; i++) {
      if (!LIDS[i]->enabled()) continue;
      const char* f = LIDS[i]->fault();
      chk(LIDS[i]->name, true, _lidOk[i] && !f, f ? f : _lidMsg[i], all);
    }
    for (Pump* pm : PUMPS) { char k[16]; snprintf(k, sizeof k, "pump_%s", pm->name); chk(k, ENABLE_PUMPS, pm->calibrated(), pm->calibrated() ? "OK" : "UNCALIBRATED", all); }
    J.close('}').kvb("ok", all).kvf("lamp_vis_rise", _visRise, 0).kvf("lamp_nir_rise", _nirRise, 0).kvb("motion", _motion);
    emit();
  }
  St _st = T_IDLE; bool _motion = false, _started = false, _lampOk = true, _lampRan = false;
  uint8_t _frame = 0, _lidI = 0, _lidPhase = 0; int32_t _lidPos0 = 0;
  uint32_t _t0 = 0, _ts = 0; uint16_t _raw[3][RF::NB];
  float _visRise = 0, _nirRise = 0; const char* _lampMsg = "SKIPPED";
  const char* _lidMsg[N_BAYS]; bool _lidOk[N_BAYS];
};
SelfTest selftest;
