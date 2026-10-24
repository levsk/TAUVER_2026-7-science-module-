#pragma once
// ============================================================================
//  swir.h — SWIR hydration add-on (STRETCH, off unless TAUVER_SWIR = 1)
//
//  Why: water and OH in minerals absorb at 1.45 um. Silicon (the Triad) is
//  blind past ~1.0 um, so wet-looking sand (darker everywhere, E11) can't be
//  told from dark sand. One InGaAs photodiode + two LEDs settles it:
//      R(1300) = continuum (no water band)      R(1450) = inside the water band
//      BD1450 = 1 - R1450 / R1300   ~0 dry,   > ~0.05 hydrated or wet
//  Same rules as reflect.h, scaled down to two channels: WHITE on the PTFE
//  first, every lit frame bracketed by darks, the white kept in RAM.
// ============================================================================
#include "subsystem.h"

class Swir : public Subsystem {
 public:
  Swir() : Subsystem("swir", ENABLE_SWIR, false) {}
  bool init() override {
    pinOutLow(PIN_SWIR_1300); pinOutLow(PIN_SWIR_1450); pinMode(PIN_SWIR_PD, INPUT_ANALOG);
    _proc = P_IDLE; _t = millis() - 1000;
    return true;
  }
  void poll() override {
    if (_proc == P_IDLE) {                           // idle: a dark reading must sit inside the ADC range
      if (since(_t) < 1000) return;
      _t = millis(); float v = readV();
      if (v < 3.2f) markOk(); else markFail("PD_SATURATED_IN_DARK");
      return;
    }
    run();
  }
  void safe() override { if (_proc != P_IDLE) abort("SUBSYSTEM_OFFLINE"); lampsOff(); }
  void toJson(Json& j) override { j.kvs("swir", PROC_NAME[_proc]); }

  bool startWhite(const char*& why)                  { return begin(P_WHITE, "", why); }
  bool startScan(const char* tag, const char*& why)  {
    if (!(_wht[0] > 0 && _wht[1] > 0)) { why = "NO_SWIR_WHITE_TAKE_WHITE_FIRST"; return false; }
    return begin(P_SCAN, tag, why);
  }
  void abort(const char* why) {
    lampsOff();
    if (_proc != P_IDLE) { _err = why; logEvent("warn", "SWIR_ABORTED", why); }
    _proc = P_IDLE; _ok = false; _t = millis();
  }
  bool busy() const   { return _proc != P_IDLE; }
  bool lastOk() const { return _ok; }
  const char* lastError() const { return _err ? _err : ""; }
  bool whiteValid() const { return _wht[0] > 0 && _wht[1] > 0; }

 private:
  enum Proc : uint8_t { P_IDLE, P_WHITE, P_SCAN };
  static constexpr const char* PROC_NAME[3] = { "IDLE", "WHITE", "SCAN" };
  static constexpr uint8_t NFR = 2 * SWIR_FRAMES + 1;  // D L D L ... D
  bool begin(Proc p, const char* tag, const char*& why) {
    if (!enabled)                    { why = "SWIR_DISABLED"; return false; }
    if (!healthy())                  { why = "SWIR_OFFLINE"; return false; }
    if (busy())                      { why = "SWIR_BUSY"; return false; }
    if (!Safety::actuationAllowed()) { why = Safety::whyNot(); return false; }
    strlcpy(_tag, tag ? tag : "", sizeof _tag);
    _proc = p; _w = 0; _k = 0; _settling = false; _ok = false; _err = nullptr; _t0 = millis();
    return true;
  }
  float readV() { uint32_t a = 0; for (uint8_t i = 0; i < SWIR_OVERSAMPLE; i++) a += analogRead(PIN_SWIR_PD); return (a / (float)SWIR_OVERSAMPLE) * VDDA / 4095.0f; }
  void lampsOff() { pinSet(PIN_SWIR_1300, false); pinSet(PIN_SWIR_1450, false); }
  uint32_t lampPin(uint8_t w) const { return w == 0 ? PIN_SWIR_1300 : PIN_SWIR_1450; }
  void run() {
    if (!Safety::actuationAllowed()) { abort(Safety::whyNot()); return; }
    if (since(_t0) > 10000)          { abort("SWIR_TIMEOUT"); return; }
    bool lit = _k & 1;
    if (lit && !_settling) { pinSet(lampPin(_w), true); _settling = true; _ts = millis(); return; }
    if (lit && since(_ts) < SWIR_SETTLE_MS) return;
    _fr[_k] = readV();
    if (lit) { pinSet(lampPin(_w), false); _settling = false; }
    if (++_k < NFR) return;
    // one wavelength done: net = lit - mean(dark before, dark after)
    float s = 0, s2 = 0;
    for (uint8_t i = 1; i < NFR; i += 2) { float n = _fr[i] - 0.5f * (_fr[i - 1] + _fr[i + 1]); s += n; s2 += n * n; }
    float m = s / SWIR_FRAMES, var = fmaxf(0.0f, s2 / SWIR_FRAMES - m * m);
    _sig[_w] = m; _sigSe[_w] = sqrtf(var / SWIR_FRAMES);
    _k = 0;
    if (++_w < 2) return;
    finish();
  }
  void finish() {
    if (_proc == P_WHITE) {
      for (uint8_t w = 0; w < 2; w++) if (_sig[w] < SWIR_MIN_SIGNAL_V) { abort(w == 0 ? "SWIR_1300_DARK" : "SWIR_1450_DARK"); return; }
      _wht[0] = _sig[0]; _wht[1] = _sig[1]; _wAt = millis();
      J.begin("result").kvs("kind", "swir_white").kvu("ms", millis()).kvf("w1300_v", _wht[0], 4).kvf("w1450_v", _wht[1], 4)
       .kvf("sw1300_v", _sigSe[0], 4).kvf("sw1450_v", _sigSe[1], 4).kvu("frames", SWIR_FRAMES);
      emit();
    } else {
      float r0 = _sig[0] / _wht[0], r1 = _sig[1] / _wht[1];
      float sr0 = _sigSe[0] / _wht[0], sr1 = _sigSe[1] / _wht[1];
      float bd = (r0 > 0.01f) ? 1.0f - r1 / r0 : NAN;
      float sbd = (r0 > 0.01f) ? (r1 / r0) * sqrtf((sr1 / fmaxf(r1, 1e-6f)) * (sr1 / fmaxf(r1, 1e-6f)) + (sr0 / r0) * (sr0 / r0)) : NAN;
      J.begin("result").kvs("kind", "swir").kvu("ms", millis()).kvs("tag", _tag).kvf("r1300", r0, 4).kvf("r1450", r1, 4)
       .kvf("sr1300", sr0, 4).kvf("sr1450", sr1, 4).kvf("bd1450", bd, 4).kvf("sbd1450", sbd, 4)
       .kvu("white_age_s", since(_wAt) / 1000).kvu("frames", SWIR_FRAMES);
      emit();
    }
    lampsOff(); _proc = P_IDLE; _ok = true; _t = millis();
  }
  Proc _proc = P_IDLE; uint8_t _w = 0, _k = 0; bool _settling = false, _ok = false;
  uint32_t _t = 0, _t0 = 0, _ts = 0, _wAt = 0;
  float _fr[NFR], _sig[2] = { 0, 0 }, _sigSe[2] = { 0, 0 }, _wht[2] = { 0, 0 };
  char _tag[24] = ""; const char* _err = nullptr;
};
constexpr const char* Swir::PROC_NAME[3];
Swir swir;
