#pragma once
// ============================================================================
//  actuators.h — THE QUICKSAND LAYER: things that move, pump or blow.
//  Every actuator: starts non-blocking, stops itself on a timer, and checks
//  Safety on EVERY poll. SAFE / E-STOP win, always, within one loop pass.
//    Pump       — H-bridge IN/IN peristaltic pump: soft start, hard time cap
//    Output     — plain timed MOSFET output (purge fan/valve, ring light)
//    StepperLid — 28BYJ-48 + ULN2003 sliding lid, optional end-stop homing
//  (The reflectance lamps live in reflect.h: Illum.)
// ============================================================================
#include "state.h"

// ----------------------------------------------------------------------------
// PUMP — "IN/IN" H-bridge (DRV8871 style): IN1 PWM = forward, IN2 PWM = reverse.
// A single-MOSFET pump works too: wire IN1 only and set IN2 = NO_PIN.
// ----------------------------------------------------------------------------
class Pump {
 public:
  enum Dir : uint8_t { STOP, FWD, REV };
  Pump(const char* n, const char* module, uint8_t calIdx, uint32_t in1, uint32_t in2)
    : name(n), module(module), _ci(calIdx), _in1(in1), _in2(in2) {}
  const char* name; const char* module;   // module = "general" (acid) or "astrobio" (liquid)
  void init() { pwmSet(_in1, 0); pwmSet(_in2, 0); }
  bool run(Dir d, uint8_t duty, uint32_t ms, const char*& why) {
    if (!ENABLE_PUMPS)                { why = "PUMPS_DISABLED"; return false; }
    if (!Safety::actuationAllowed())  { why = Safety::whyNot(); return false; }
    if (d == STOP)                    { stop(); return true; }
    if (d == REV && _in2 == NO_PIN)   { why = "NO_REVERSE_WIRED"; return false; }
    if (ms == 0 || ms > PUMP_MAX_RUN_MS) { why = "BAD_DURATION"; return false; }
    if (duty == 0) duty = PUMP_DEFAULT_DUTY;
    stop();
    _dir = d; _target = duty; _duty = 0; _runMs = ms; _t0 = millis(); _busy = true;
    return true;
  }
  bool doseMl(float ml, const char*& why) {             // volume dose: needs CMD:PUMP_CAL first
    float r = cal.pumpMlps[_ci];
    if (!(r > 0.0f))   { why = "PUMP_UNCALIBRATED"; return false; }
    if (!(ml > 0.0f))  { why = "BAD_VOLUME"; return false; }
    return run(FWD, PUMP_DEFAULT_DUTY, (uint32_t)(ml / r * 1000.0f + 0.5f), why);
  }
  void stop() {
    pwmSet(_in1, 0); pwmSet(_in2, 0);
    if (_busy) { _lastRunMs = since(_t0); _lastDir = _dir; _busy = false; }
    _dir = STOP; _duty = 0;
  }
  void poll() {
    if (!_busy) return;
    if (!Safety::actuationAllowed()) { stop(); return; }
    uint32_t e = since(_t0);
    if (e >= _runMs) { stop(); return; }
    uint8_t d = (e >= PUMP_RAMP_MS) ? _target : (uint8_t)((uint32_t)_target * e / PUMP_RAMP_MS);
    if (d != _duty) { _duty = d; apply(); }
  }
  // calibration: run FORWARD for >= 1 s at the default duty into a measuring cylinder,
  // then send CMD:PUMP_CAL=<name>,<mL measured>.
  bool calibrate(float measuredMl, const char*& why) {
    if (_lastDir != FWD || _lastRunMs < 1000) { why = "RUN_FWD_1S_FIRST"; return false; }
    if (!(measuredMl > 0.0f))                  { why = "BAD_VOLUME"; return false; }
    cal.pumpMlps[_ci] = measuredMl * 1000.0f / (float)_lastRunMs;
    calSave(); return true;
  }
  bool busy() const        { return _busy; }
  Dir  dir() const         { return _dir; }
  bool calibrated() const  { return cal.pumpMlps[_ci] > 0.0f; }
  float mlps() const       { return cal.pumpMlps[_ci]; }
  uint32_t lastRunMs() const { return _lastRunMs; }
  Dir  lastDir() const     { return _lastDir; }
  float lastMl() const     { return calibrated() && _lastDir == FWD ? mlps() * _lastRunMs / 1000.0f : 0.0f; }   // what the LAST forward run moved
  const char* dirName() const { return _dir == FWD ? "FWD" : _dir == REV ? "REV" : "OFF"; }
 private:
  void apply() {
    if (_dir == FWD)      { pwmSet(_in2, 0); pwmSet(_in1, _duty); }
    else if (_dir == REV) { pwmSet(_in1, 0); pwmSet(_in2, _duty); }
  }
  uint8_t _ci; uint32_t _in1, _in2;
  Dir _dir = STOP, _lastDir = STOP; uint8_t _target = 0, _duty = 0; bool _busy = false;
  uint32_t _runMs = 0, _t0 = 0, _lastRunMs = 0;
};

// ----------------------------------------------------------------------------
// OUTPUT — a timed digital MOSFET output (purge fan / valve, microscope light).
// ----------------------------------------------------------------------------
class Output {
 public:
  Output(const char* n, uint32_t pin, bool enabled) : name(n), _pin(pin), _en(enabled) {}
  const char* name;
  void init() { pinOutLow(_pin); }
  bool on(uint32_t ms, const char*& why) {       // ms = 0 -> stays on until off()
    if (!_en || _pin == NO_PIN)       { why = "DISABLED"; return false; }
    if (!Safety::actuationAllowed())  { why = Safety::whyNot(); return false; }
    digitalWrite(_pin, HIGH); _on = true; _t0 = millis(); _ms = ms; return true;
  }
  void off()  { pinSet(_pin, false); _on = false; }
  void poll() { if (_on && (!Safety::actuationAllowed() || (_ms && since(_t0) >= _ms))) off(); }
  bool isOn() const    { return _on; }
  bool enabled() const { return _en && _pin != NO_PIN; }
 private:
  uint32_t _pin; bool _en, _on = false; uint32_t _t0 = 0, _ms = 0;
};

// ----------------------------------------------------------------------------
// STEPPER LID — 28BYJ-48 via ULN2003, full-step two-phase (both coils on =
// most torque). Coils are released after every move: an energised 28BYJ burns
// ~0.5 W per coil and the gearbox holds the lid anyway.
// Position is open-loop. Without an end-stop the firmware ASSUMES the lid is
// closed at boot (reported "homed":0). With an end-stop, CMD:LID=..,HOME.
// ----------------------------------------------------------------------------
class StepperLid {
 public:
  enum State : uint8_t { CLOSED, OPEN, MOVING, MIDWAY, HOMING };
  StepperLid(const char* n, uint8_t idx, const uint32_t* pins, uint32_t endstop, bool enabled)
    : name(n), _i(idx), _p(pins), _end(endstop), _en(enabled) {}
  const char* name;
  static StepperLid* active;                       // one lid moves at a time (current budget)
  void init() {
    if (!_en) return;
    for (uint8_t k = 0; k < 4; k++) pinOutLow(_p[k]);
    if (_end != NO_PIN) pinMode(_end, INPUT_PULLUP);
    _pos = 0; _target = 0; _moving = false; _homed = false;
  }
  bool enabled() const { return _en; }
  bool open(const char*& why)  { return go(LID_TRAVEL_STEPS[_i], false, why); }
  bool close(const char*& why) {
    if (LID_CLOSE_NEEDS_ARM_CLEAR && !armIsClear()) { why = "ARM_NOT_CLEAR"; return false; }
    return go(0, false, why);
  }
  bool jog(int32_t steps, const char*& why) { return go(constrain(_pos + steps, (int32_t)0, LID_TRAVEL_STEPS[_i]), false, why); }
  bool home(const char*& why) {
    if (_end == NO_PIN) { why = "NO_ENDSTOP_WIRED"; return false; }
    return go(_pos - (LID_TRAVEL_STEPS[_i] * 5) / 4, true, why);   // drive closed until the switch
  }
  void stop() { _moving = false; _homing = false; release(); if (active == this) active = nullptr; }
  void poll() {
    if (!_moving) return;
    if (!Safety::actuationAllowed()) { stop(); _fault = "STOPPED_BY_SAFE"; return; }
    if (sinceUs(_lastStepUs) < LID_STEP_US) return;
    _lastStepUs = micros();
    if (_homing && endstopHit()) { _pos = 0; _target = 0; _homed = true; stop(); return; }
    if (_pos == _target) {
      if (_homing) { _fault = "HOME_SWITCH_NOT_FOUND"; logEvent("error", "LID_HOME_FAIL", name); }
      stop(); return;
    }
    int8_t dir = _target > _pos ? 1 : -1;
    _pos += dir;
    _phase = (uint8_t)((_phase + (LID_INVERT[_i] ? -dir : dir) + 4) & 3);
    energize(_phase);
  }
  State state() const {
    if (_homing) return HOMING;
    if (_moving) return MOVING;
    if (_pos <= 0) return CLOSED;
    if (_pos >= LID_TRAVEL_STEPS[_i]) return OPEN;
    return MIDWAY;
  }
  const char* stateName() const { static const char* const N[] = { "CLOSED", "OPEN", "MOVING", "MIDWAY", "HOMING" }; return _en ? N[state()] : "NONE"; }
  bool isClosed() const { return !_en || state() == CLOSED; }      // no lid fitted = nothing to be open
  bool moving() const   { return _moving; }
  bool homed() const    { return _homed; }
  int32_t pos() const   { return _pos; }
  const char* fault() const { return _fault; }
 private:
  bool go(int32_t target, bool homing, const char*& why) {
    if (!ENABLE_LIDS || !_en)        { why = "LID_NOT_FITTED"; return false; }
    if (!Safety::actuationAllowed()) { why = Safety::whyNot(); return false; }
    if (active && active != this)    { why = "OTHER_LID_MOVING"; return false; }
    _target = target; _homing = homing; _fault = nullptr;
    if (_pos == _target && !homing) { return true; }        // already there
    _moving = true; active = this; _lastStepUs = micros() - LID_STEP_US;
    energize(_phase);                                       // hold the current phase first
    return true;
  }
  bool endstopHit() const { return _end != NO_PIN && digitalRead(_end) == LOW; }
  void energize(uint8_t ph) {
    static const uint8_t SEQ[4] = { 0b1100, 0b0110, 0b0011, 0b1001 };
    for (uint8_t k = 0; k < 4; k++) digitalWrite(_p[k], (SEQ[ph] >> (3 - k)) & 1 ? HIGH : LOW);
  }
  void release() { if (_en) for (uint8_t k = 0; k < 4; k++) digitalWrite(_p[k], LOW); }
  uint8_t _i; const uint32_t* _p; uint32_t _end; bool _en;
  int32_t _pos = 0, _target = 0; uint8_t _phase = 0;
  bool _moving = false, _homing = false, _homed = false;
  uint32_t _lastStepUs = 0; const char* _fault = nullptr;
};
StepperLid* StepperLid::active = nullptr;
