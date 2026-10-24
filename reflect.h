#pragma once
// ============================================================================
//  reflect.h — THE REFLECTANCE INSTRUMENT
//  Shared byte-for-byte by the flight firmware 3.0 and the Reflectance Bench:
//  every hour on the bench tests flight code.
//
//  What it measures: R(λ) = (sample rate) / (white rate), band by band, where
//  a "rate" is counts per millisecond per gain unit with lamp-off frames
//  subtracted. R is a property of the SAMPLE alone: the lamp spectrum, the
//  filters and the silicon all cancel in the ratio.
//
//    Illum      the lamp groups: MOSFET gates + the chip's own LEDs
//    TriadDrv   AS7265x "Triad" @0x49: 18 bands 410-940 nm, virtual-register mailbox
//    As7341Drv  AS7341 @0x39: 8 visible bands + 910 nm, SMUX halves, onboard LED
//    Reflect    the Subsystem: LIVE, WHITE, SCAN, LAMP test, the white in flash
//
//  Six rules (Logic Sweep 3.0):
//   1. 45°/0° geometry (the probe head), so a flat face's glint misses the sensor.
//   2. WHITE = PTFE in the sample's exact place.
//   3. Integration time is the exposure knob (exact). Gain is only nominal, so
//      white and sample always use the SAME gain per lamp group.
//   4. Bracketed darks: a lamp-off frame before AND after every lit frame
//      cancels stray light and slow drift.
//   5. Bands a group barely lights, and saturated bands, come back null.
//      Noise divided by noise is not data.
//   6. Only ratios and band depths are science; absolute brightness moves with
//      height (1/d²).
//  The sketch provides:   bool reflectAllowed();   (may lamps fire right now?)
// ============================================================================
#include "subsystem.h"

bool reflectAllowed();

namespace RF {
  constexpr uint8_t NB = 18;                         // most bands any chip has
  constexpr uint8_t LED_WHITE = 1, LED_IR = 2, LED_UV = 4;   // chip LED bits (Triad die 0 / 1 / 2; AS7341: WHITE)
  enum Chip : uint8_t { CHIP_NONE, CHIP_TRIAD, CHIP_AS7341 };
  const char* const CHIP_NAME[] = { "NONE", "AS7265X", "AS7341" };
}

// ============================================================================
// Illum — the lamp groups. A group = one MOSFET gate (or NO_PIN) + chip LEDs.
// Lamps only burn while a frame needs them, and never longer than the cap.
// ============================================================================
class Illum {
 public:
  void begin() { for (uint8_t g = 0; g < N_ILLUM; g++) pinOutLow(PIN_ILLUM[g]); _mask = 0; }
  bool on(uint8_t mask, uint32_t capMs, const char*& why) {
    if (!reflectAllowed()) { why = "LAMPS_NOT_ALLOWED"; return false; }
    for (uint8_t g = 0; g < N_ILLUM; g++) pinSet(PIN_ILLUM[g], (mask >> g) & 1);
    if (!_mask && mask) _t0 = millis();
    _mask = mask; _cap = capMs;
    return true;
  }
  void off() {
    for (uint8_t g = 0; g < N_ILLUM; g++) pinSet(PIN_ILLUM[g], false);
    if (_mask) _total += since(_t0);
    _mask = 0;
  }
  void poll() {                                      // the cap and the safety rule never sleep
    if (!_mask) return;
    if (!reflectAllowed() || (_cap && since(_t0) > _cap)) { off(); _forced = true; }
  }
  uint8_t  mask() const    { return _mask; }
  uint32_t onMs() const    { return _mask ? since(_t0) : 0; }
  uint32_t totalMs() const { return _total + onMs(); }
  bool takeForced()        { bool f = _forced; _forced = false; return f; }
 private:
  uint8_t _mask = 0; uint32_t _t0 = 0, _cap = 0, _total = 0; bool _forced = false;
};
Illum illum;

// ============================================================================
// SpecDrv — what the Reflect procedures need from a chip. A frame is started,
// then polled; poll() does a bounded slice of bus work and returns.
// ============================================================================
class SpecDrv {
 public:
  uint8_t nb = 0; const uint16_t* wl = nullptr; RF::Chip chip = RF::CHIP_NONE;
  uint16_t raw[RF::NB]; bool sat = false; const char* err = "";
  virtual bool  begin() = 0;                                  // detect + configure (a few ms, sync)
  virtual bool  alive() = 0;                                  // cheap "still there?"
  virtual uint8_t nGain() const = 0;
  virtual float gainF(uint8_t i) const = 0;                   // nominal gain factor
  virtual uint8_t gainStart() const = 0;
  virtual float tintQuant(float ms) const = 0;                // nearest achievable integration time
  virtual float tintMin() const = 0;
  virtual float tintMax() const = 0;
  virtual float fullScale(float tintMs) const = 0;
  virtual void  start(uint8_t gain, float tintMs, uint8_t chipLeds) = 0;
  virtual int8_t poll() = 0;                                  // 0 running · 1 done (raw[] valid) · -1 error (err)
  virtual void  ledsOff() = 0;                                // best effort, sync
  virtual ~SpecDrv() {}
};

// ============================================================================
// TriadDrv — SparkFun Triad (AS72651 master + AS72652 + AS72653 slaves).
// The master hides its registers behind a mailbox:
//   physical 0x00 STATUS (bit0 RX_VALID, bit1 TX_VALID) · 0x01 WRITE · 0x02 READ
//   virtual write: wait TX_VALID=0, WRITE <- reg|0x80, wait TX_VALID=0, WRITE <- data
//   virtual read : (drain READ if RX_VALID), wait TX_VALID=0, WRITE <- reg, wait RX_VALID=1, READ
// Each virtual op is bounded (VOP_TMO_US); a frame is ~50 of them, done a few
// per loop pass so the loop stays fast.
// ============================================================================
class TriadDrv : public SpecDrv {
 public:
  TriadDrv() { nb = 18; wl = WL; chip = RF::CHIP_TRIAD; }
  bool begin() override {
    err = "";
    if (!Bus::probe(ADDR_TRIAD)) { err = "NO_ACK"; return false; }
    uint8_t sel = 0;
    // The Triad keeps power when the MCU resets, so its mailbox may be half-way
    // through a write: our first read request then lands as that write's DATA
    // byte and never answers. A second read starts clean (bench finding 3.0).
    if (!vRead(V_DEVSEL, sel) && !vRead(V_DEVSEL, sel)) { err = "MAILBOX_NO_ANSWER"; return false; }
    if ((sel & 0x30) != 0x30) { err = "SLAVES_MISSING"; return false; }
    // Put every register we use into a known state (this also repairs any
    // register that stray byte may have hit).
    for (uint8_t d = 0; d < 3; d++) if (!vWrite(V_DEVSEL, d) || !vWrite(V_LED, 0)) { err = "CONFIG_WRITE"; return false; }
    if (!vWrite(V_INT, 18)) { err = "CONFIG_WRITE"; return false; }
    _int = 18; _leds = 0; _st = T_IDLE;
    return true;
  }
  bool alive() override { uint8_t s; return Bus::readRegStop(ADDR_TRIAD, P_STATUS, &s, 1); }
  uint8_t nGain() const override { return 4; }
  float gainF(uint8_t i) const override { static const float G[4] = { 1.0f, 3.7f, 16.0f, 64.0f }; return G[i < 4 ? i : 3]; }
  uint8_t gainStart() const override { return 2; }
  float tintQuant(float ms) const override { int n = (int)lroundf(ms / 2.8f); n = n < 1 ? 1 : (n > 255 ? 255 : n); return n * 2.8f; }
  float tintMin() const override { return 2.8f; }
  float tintMax() const override { return 255 * 2.8f; }
  float fullScale(float) const override { return 65535.0f; }
  void start(uint8_t gain, float tintMs, uint8_t chipLeds) override {
    _gain = gain < 4 ? gain : 3; _intWant = (uint8_t)lroundf(tintQuant(tintMs) / 2.8f);
    _ledWant = chipLeds; _die = 0; _k = 0; sat = false; err = ""; _st = T_INT;
  }
  int8_t poll() override {                           // bus work in slices of ~2.5 ms, so the loop stays fast
    uint32_t t0 = micros();
    do {
      int8_t r = stepOnce();
      if (r != 0 || _st == T_WAIT) return r;          // waiting = no bus work this pass
    } while (sinceUs(t0) < SLICE_US);
    return 0;
  }
  void ledsOff() override { for (uint8_t d = 0; d < 3; d++) { vWrite(V_DEVSEL, d); vWrite(V_LED, 0); } _leds = 0; }

 private:
  static constexpr uint8_t P_STATUS = 0x00, P_WRITE = 0x01, P_READ = 0x02, TXV = 0x02, RXV = 0x01;
  static constexpr uint8_t V_CONFIG = 0x04, V_INT = 0x05, V_LED = 0x07, V_DATA = 0x08, V_DEVSEL = 0x4F;
  static constexpr uint32_t SLICE_US = 2500;      // a virtual op is ~1-2 ms at 100 kHz: one slice ~ 1-2 ops
  static constexpr uint32_t VOP_TMO_US = 10000;
  static const uint16_t WL[18]; static const uint8_t DIE[18], SLOT[18];
  enum St : uint8_t { T_IDLE, T_INT, T_LED, T_START, T_WAIT, T_READ, T_DONE };

  bool fail(const char* why) { err = why; return false; }
  bool physR(uint8_t reg, uint8_t& v) { return Bus::readRegStop(ADDR_TRIAD, reg, &v, 1); }
  bool physW(uint8_t reg, uint8_t v)  { return Bus::writeReg(ADDR_TRIAD, reg, v); }
  bool waitSt(uint8_t bit, bool want) {
    uint32_t t0 = micros();
    for (;;) {
      uint8_t s; if (!physR(P_STATUS, s)) return fail("I2C");
      if (((s & bit) != 0) == want) return true;
      if (sinceUs(t0) > VOP_TMO_US) return fail(bit == TXV ? "TX_BUSY" : "RX_TIMEOUT");
      delayMicroseconds(40);
    }
  }
  bool vWrite(uint8_t reg, uint8_t d) {
    return waitSt(TXV, false) && (physW(P_WRITE, reg | 0x80) || fail("I2C"))
        && waitSt(TXV, false) && (physW(P_WRITE, d) || fail("I2C"));
  }
  bool vRead(uint8_t reg, uint8_t& d) {
    uint8_t s; if (!physR(P_STATUS, s)) return fail("I2C");
    if (s & RXV) { uint8_t junk; physR(P_READ, junk); }          // drain a stale answer
    return waitSt(TXV, false) && (physW(P_WRITE, reg) || fail("I2C"))
        && waitSt(RXV, true) && (physR(P_READ, d) || fail("I2C"));
  }
  // one virtual op (or one wait check) per call
  int8_t stepOnce() {
    switch (_st) {
      case T_INT:
        if (_intWant != _int) { if (!vWrite(V_INT, _intWant)) return -1; _int = _intWant; }
        _st = T_LED; return 0;
      case T_LED: {
        uint8_t diff = (_ledWant ^ _leds) & 7;
        if (!diff) { _st = T_START; return 0; }
        uint8_t d = (diff & 1) ? 0 : (diff & 2) ? 1 : 2;              // bit0 white=die0, bit1 IR=die1, bit2 UV=die2
        bool on = (_ledWant >> d) & 1;
        if (!vWrite(V_DEVSEL, d) || !vWrite(V_LED, on ? (uint8_t)(0x08 | (TRIAD_LED_CUR << 4)) : 0)) return -1;
        _leds = (uint8_t)((_leds & ~(1 << d)) | (on << d));
        return 0;
      }
      case T_START:                                                  // BANK 3 = all six channels, one shot
        if (!vWrite(V_CONFIG, (uint8_t)((_gain << 4) | (3 << 2)))) return -1;
        _t0 = millis(); _st = T_WAIT; return 0;
      case T_WAIT: {
        uint32_t need = (uint32_t)(2.0f * _int * 2.8f);              // two banks of one integration each
        if (since(_t0) < need) return 0;
        uint8_t c; if (!vRead(V_CONFIG, c)) return -1;
        if (c & 0x02) { _st = T_READ; _die = 0; _k = 0; _selDone = false; return 0; }
        if (since(_t0) > need + 300) { err = "DATA_TIMEOUT"; return -1; }
        return 0;
      }
      case T_READ: {                                                 // one data byte per step (12 per die, MSB first)
        if (_k == 0 && !_selDone) { if (!vWrite(V_DEVSEL, _die)) return -1; _selDone = true; return 0; }
        uint8_t v; if (!vRead((uint8_t)(V_DATA + _k), v)) return -1;
        uint16_t& w = _buf[_die][_k >> 1];
        w = (_k & 1) ? (uint16_t)((w & 0xFF00) | v) : (uint16_t)(v << 8);
        if (++_k >= 12) { _k = 0; _selDone = false; if (++_die >= 3) { finishFrame(); _st = T_DONE; return 1; } }
        return 0;
      }
      default: return _st == T_DONE ? 1 : 0;
    }
  }
  void finishFrame() {
    for (uint8_t b = 0; b < 18; b++) { raw[b] = _buf[DIE[b]][SLOT[b]]; if (raw[b] >= 65535) sat = true; }
  }
  St _st = T_IDLE; uint8_t _gain = 2, _int = 0, _intWant = 18, _leds = 0, _ledWant = 0, _die = 0, _k = 0;
  uint32_t _t0 = 0; uint16_t _buf[3][6]; bool _selDone = false;
};
// band order (ascending wavelength) -> die (0 = AS72651 "NIR" RSTUVW, 1 = AS72652 "VIS" GHIJKL, 2 = AS72653 "UV" ABCDEF) and slot
const uint16_t TriadDrv::WL[18]  = { 410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940 };
const uint8_t  TriadDrv::DIE[18]  = {   2,   2,   2,   2,   2,   2,   1,   1,   0,   1,   0,   1,   0,   0,   0,   0,   1,   1 };
const uint8_t  TriadDrv::SLOT[18] = {   0,   1,   2,   3,   4,   5,   0,   1,   0,   2,   1,   3,   2,   3,   4,   5,   4,   5 };

// ============================================================================
// As7341Drv — 8 visible filters + 910 nm "NIR". Six ADCs for ten photodiodes,
// so a frame is two SMUX "halves": low = F1-F4 + Clear + NIR, high = F5-F8.
// Its onboard LED (if the breakout has one) sits behind register bank 1.
// SMUX tables from Adafruit_AS7341 (BSD), checked against DS000504.
// ============================================================================
class As7341Drv : public SpecDrv {
 public:
  As7341Drv() { nb = 9; wl = WL; chip = RF::CHIP_AS7341; }
  bool begin() override {
    err = "";
    uint8_t id = 0;
    if (!Bus::readReg(ADDR_AS7341, 0x92, &id, 1)) { err = "NO_ACK"; return false; }
    if ((id & 0xFC) != 0x24)                       { err = "BAD_CHIP_ID"; return false; }
    if (!Bus::writeReg(ADDR_AS7341, 0x80, 0x01))   { err = "POWER_ON"; return false; }
    _ledSet = 0xFF; if (!setLed(false)) { err = "LED_CONFIG"; return false; }
    _st = A_IDLE; return true;
  }
  bool alive() override { uint8_t id; return Bus::readReg(ADDR_AS7341, 0x92, &id, 1) && (id & 0xFC) == 0x24; }
  uint8_t nGain() const override { return 11; }
  float gainF(uint8_t i) const override { return 0.5f * (float)(1u << (i < 11 ? i : 10)); }
  uint8_t gainStart() const override { return 5; }                                       // 16x
  float tintQuant(float ms) const override { int n = (int)lroundf(ms / 2.78f); n = n < 1 ? 1 : (n > 256 ? 256 : n); return n * 2.78f; }
  float tintMin() const override { return 2.78f; }
  float tintMax() const override { return 256 * 2.78f; }
  float fullScale(float tintMs) const override { float n = tintQuant(tintMs) / 2.78f * 1000.0f; return n > 65535.0f ? 65535.0f : n; }
  void start(uint8_t gain, float tintMs, uint8_t chipLeds) override {
    _gain = gain < 11 ? gain : 10; _atime = (uint8_t)(lroundf(tintQuant(tintMs) / 2.78f) - 1);
    _ledWant = chipLeds & RF::LED_WHITE; _half = 0; sat = false; err = ""; _st = A_CFG;
  }
  int8_t poll() override {
    switch (_st) {
      case A_CFG:
        if (!setLed(_ledWant) || !Bus::writeReg(ADDR_AS7341, 0x81, _atime) || !Bus::writeReg(ADDR_AS7341, 0xCA, 0xE7)
            || !Bus::writeReg(ADDR_AS7341, 0xCB, 0x03) || !Bus::writeReg(ADDR_AS7341, 0xAA, _gain)) return fault("CONFIG_WRITE");
        _st = A_SMUX_START; return 0;
      case A_SMUX_START: {
        uint8_t b[21]; b[0] = 0x00; memcpy(b + 1, _half == 0 ? SMUX_LOW : SMUX_HIGH, 20);
        if (!Bus::writeReg(ADDR_AS7341, 0x80, 0x01) || !Bus::writeReg(ADDR_AS7341, 0xAF, 0x10)
            || !Bus::write(ADDR_AS7341, b, 21) || !Bus::writeReg(ADDR_AS7341, 0x80, 0x11)) return fault("SMUX_WRITE");
        _t = millis(); _st = A_SMUX_WAIT; return 0;
      }
      case A_SMUX_WAIT: {
        uint8_t en; if (!Bus::readReg(ADDR_AS7341, 0x80, &en, 1)) return fault("I2C");
        if (en & 0x10) { if (since(_t) > 50) return fault("SMUX_TIMEOUT"); return 0; }
        if (!Bus::writeReg(ADDR_AS7341, 0x80, 0x03)) return fault("I2C");                // PON + SP_EN: integrate
        _t = millis(); _st = A_INT; return 0;
      }
      case A_INT: {
        float tint = (_atime + 1) * 2.78f;
        if (since(_t) + 2 < (uint32_t)tint) return 0;
        uint8_t s2; if (!Bus::readReg(ADDR_AS7341, 0xA3, &s2, 1)) return fault("I2C");
        if (!(s2 & 0x40)) { if (since(_t) > (uint32_t)tint + 500) return fault("DATA_TIMEOUT"); return 0; }
        uint8_t d[13]; if (!Bus::readReg(ADDR_AS7341, 0x94, d, 13)) return fault("I2C");
        if (d[0] & 0x80) sat = true;
        uint16_t ch[6]; for (uint8_t i = 0; i < 6; i++) ch[i] = (uint16_t)(d[1 + 2 * i] | (d[2 + 2 * i] << 8));
        Bus::writeReg(ADDR_AS7341, 0x80, 0x01);
        float fs = fullScale(tint);
        if (_half == 0) { for (uint8_t i = 0; i < 4; i++) raw[i] = ch[i]; raw[8] = ch[5]; if (ch[5] >= fs) sat = true; }
        else            { for (uint8_t i = 0; i < 4; i++) raw[4 + i] = ch[i]; }
        for (uint8_t i = 0; i < 4; i++) if (ch[i] >= fs) sat = true;
        if (++_half < 2) { _st = A_SMUX_START; return 0; }
        _st = A_DONE; return 1;
      }
      case A_DONE: return 1;
      default: return 0;
    }
  }
  void ledsOff() override { setLed(false); }
 private:
  enum St : uint8_t { A_IDLE, A_CFG, A_SMUX_START, A_SMUX_WAIT, A_INT, A_DONE };
  int8_t fault(const char* why) { err = why; _st = A_IDLE; return -1; }
  bool setLed(bool on) {                       // bank 1: CONFIG 0x70 (LED_SEL) + LED 0x74 (LED_ACT | drive)
    if (_ledSet == (uint8_t)on) return true;
    uint8_t drive = (uint8_t)((AS7341_LED_MA > 4 ? (AS7341_LED_MA - 4) / 2 : 0) & 0x7F);
    bool ok = Bus::writeReg(ADDR_AS7341, 0xA9, 0x10) && Bus::writeReg(ADDR_AS7341, 0x70, 0x08)
           && Bus::writeReg(ADDR_AS7341, 0x74, (uint8_t)((on ? 0x80 : 0x00) | drive)) && Bus::writeReg(ADDR_AS7341, 0xA9, 0x00);
    if (ok) _ledSet = on;
    return ok;
  }
  static const uint16_t WL[9]; static const uint8_t SMUX_LOW[20], SMUX_HIGH[20];
  St _st = A_IDLE; uint8_t _gain = 5, _atime = 17, _half = 0, _ledWant = 0, _ledSet = 0xFF; uint32_t _t = 0;
};
const uint16_t As7341Drv::WL[9] = { 415, 445, 480, 515, 555, 590, 630, 680, 910 };
const uint8_t As7341Drv::SMUX_LOW[20]  = { 0x30,0x01,0x00,0x00,0x00,0x42,0x00,0x00,0x50,0x00,0x00,0x00,0x20,0x04,0x00,0x30,0x01,0x50,0x00,0x06 };
const uint8_t As7341Drv::SMUX_HIGH[20] = { 0x00,0x00,0x00,0x40,0x02,0x00,0x10,0x03,0x50,0x10,0x03,0x00,0x00,0x00,0x24,0x00,0x00,0x50,0x00,0x06 };

TriadDrv  triadDrv;
As7341Drv as7341Drv;

// ============================================================================
// The white reference, kept in flash so it survives a reset.
// ============================================================================
struct WhiteRec {
  uint32_t magic; uint16_t version, size;
  uint8_t  chip, nb, frames, groups;            // groups = mask of lamp groups captured
  uint8_t  gain[N_ILLUM], gainMoved[N_ILLUM];
  float    tint[N_ILLUM], peak[N_ILLUM];
  float    W[N_ILLUM][RF::NB], sW[N_ILLUM][RF::NB];
  uint32_t valid[N_ILLUM];                       // per group: bit b = band b is lit well enough
  uint32_t crc;
};
constexpr uint32_t WHITE_MAGIC = 0x52465731;      // "RFW1"
constexpr uint16_t WHITE_VERSION = 1;

// ============================================================================
// Reflect — the Subsystem
// ============================================================================
class Reflect : public Subsystem {
 public:
  enum Proc : uint8_t { P_IDLE, P_LIVE, P_WHITE, P_SCAN, P_LAMP };
  Reflect() : Subsystem("refl", ENABLE_REFLECT, true) {}

  // ---------------- settings (bench SET: commands; flight uses the defaults) ----------------
  uint8_t frames = REFLECT_FRAMES;               // lit frames per group in WHITE / SCAN
  uint8_t groups = REFLECT_GROUPS;               // mask of lamp groups used by WHITE / SCAN
  uint8_t manGain = 0xFF; float manTint = 0;     // LIVE exposure (0xFF = chip default)

  // ---------------- contract ----------------
  bool init() override {
    illum.off(); _proc = P_IDLE; _drv = nullptr;
    bool tryTriad = REFLECT_CHIP_PREF != 2, tryAs = REFLECT_CHIP_PREF != 1;
    if (tryTriad && triadDrv.begin()) _drv = &triadDrv;
    else if (tryTriad && REFLECT_CHIP_PREF == 0 && millis() < REFLECT_TRIAD_BOOT_MS && Bus::probe(ADDR_AS7341)) return initFail("WAIT_TRIAD_BOOT");
    else if (tryAs && as7341Drv.begin()) _drv = &as7341Drv;
    if (!_drv) return initFail(tryTriad ? (triadDrv.err[0] && strcmp(triadDrv.err, "NO_ACK") ? triadDrv.err : "NO_SPECTRAL_CHIP") : as7341Drv.err);
    if (manGain == 0xFF || manGain >= _drv->nGain()) manGain = _drv->gainStart();
    if (manTint <= 0) manTint = _drv->tintQuant(REFLECT_TINT_START_MS);
    loadWhite();
    _t = millis() - 2000;
    return true;
  }
  void poll() override {
    illum.poll();
    if (illum.takeForced() && _proc != P_IDLE) { abort(reflectAllowed() ? "LAMP_CAP" : "SAFE"); return; }
    if (_proc == P_IDLE) {                         // idle: prove the chip is alive every 2 s
      if (since(_t) > 2000) { _t = millis(); if (_drv && _drv->alive()) markOk(); else markFail("ALIVE_CHECK"); }
      return;
    }
    if (_proc == P_LAMP) { if (since(_t0) >= _lampMs) { lampOffAll(); _proc = P_IDLE; _ok = true; logEvent("info", "LAMP_TIMER_OFF"); } return; }
    run();
  }
  void safe() override { if (_proc != P_IDLE) abort("SUBSYSTEM_OFFLINE"); illum.off(); }
  void toJson(Json& j) override { j.kvs("refl", PROC_NAME[_proc]).kv("wht", whiteAgeS()); }

  // ---------------- queries ----------------
  bool busy() const               { return _proc != P_IDLE; }
  bool lastOk() const             { return _ok; }
  const char* lastError() const   { return _err ? _err : ""; }
  const char* chipName() const    { return RF::CHIP_NAME[_drv ? _drv->chip : 0]; }
  SpecDrv* drv() const            { return _drv; }
  bool whiteValid() const         { return _w.magic == WHITE_MAGIC && _drv && _w.chip == _drv->chip; }
  int32_t whiteAgeS() const       { return !whiteValid() ? -1 : (_wFresh ? (int32_t)(since(_wAt) / 1000) : -2); }  // -2 = from flash, age unknown
  const char* procName() const    { return PROC_NAME[_proc]; }
  const char* phaseName() const   { return _proc == P_WHITE || _proc == P_SCAN ? (_ae ? "AE" : "CAP") : "-"; }

  // ---------------- procedures ----------------
  bool startLive(uint8_t mask, uint32_t ms, const char*& why) {           // mask 0 = DARK
    if (!ready(why)) return false;
    _proc = P_LIVE; _liveMask = mask; _liveMs = ms ? ms : (mask ? LIVE_LIT_MAX_MS : LIVE_DARK_MAX_MS);
    if (mask && _liveMs > LIVE_LIT_MAX_MS) _liveMs = LIVE_LIT_MAX_MS;
    _t0 = millis(); _gIdx = 0; _ok = false; _err = nullptr; _nFr = 0;
    beginFrame(mask, manGain, manTint);
    return true;
  }
  bool startWhite(const char*& why) {
    if (!ready(why)) return false;
    if (!(groups & usableGroups())) { why = "NO_LAMP_GROUP"; return false; }
    memset(&_nw, 0, sizeof _nw); _nw.chip = _drv->chip; _nw.nb = _drv->nb; _nw.frames = frames; _nw.groups = groups & usableGroups();
    return beginProc(P_WHITE, why);
  }
  bool startScan(const char* tag, const char*& why) {
    if (!ready(why)) return false;
    if (!whiteValid())                   { why = _w.magic == WHITE_MAGIC ? "WHITE_FROM_OTHER_CHIP" : "NO_WHITE_TAKE_WHITE_FIRST"; return false; }
    if (!(groups & _w.groups))           { why = "NO_WHITE_FOR_THESE_GROUPS"; return false; }
    strlcpy(_tag, tag ? tag : "", sizeof _tag);
    return beginProc(P_SCAN, why);
  }
  bool lampTest(uint8_t mask, uint32_t ms, const char*& why) {
    if (!ready(why)) return false;
    if (ms == 0 || ms > REFLECT_LAMP_CAP_MS) ms = REFLECT_LAMP_CAP_MS;
    uint8_t chipLeds = 0; for (uint8_t g = 0; g < N_ILLUM; g++) if ((mask >> g) & 1) chipLeds |= ILLUM_CHIPLED[g];
    if (!illum.on(mask, REFLECT_LAMP_CAP_MS, why)) return false;
    if (chipLeds) { _drv->start(_drv->gainStart(), _drv->tintMin(), chipLeds); for (uint8_t i = 0; i < 4 && _drv->poll() == 0; i++) {} }   // first slices set the chip LEDs
    _chipLedsOn = chipLeds != 0;
    _proc = P_LAMP; _t0 = millis(); _lampMs = ms; _ok = false; _err = nullptr;
    return true;
  }
  bool clearWhite() { memset(&_w, 0, sizeof _w); Store::save(REFLECT_WHITE_STORE_OFF, &_w, sizeof _w, 0, 0); _wFresh = false; return true; }
  void abort(const char* why) {
    lampOffAll();
    if (_proc != P_IDLE) { _err = why; logEvent("warn", "REFLECT_ABORTED", why); }
    _proc = P_IDLE; _ok = false; _t = millis();
  }

 private:
  static constexpr const char* PROC_NAME[5] = { "IDLE", "LIVE", "WHITE", "SCAN", "LAMP" };

  bool ready(const char*& why) {
    if (!enabled)              { why = "REFLECT_DISABLED"; return false; }
    if (!healthy() || !_drv)   { why = "REFLECT_OFFLINE"; return false; }
    if (busy())                { why = "REFLECT_BUSY"; return false; }
    if (!reflectAllowed())     { why = "LAMPS_NOT_ALLOWED"; return false; }
    return true;
  }
  uint8_t usableGroups() const { uint8_t m = 0; for (uint8_t g = 0; g < N_ILLUM; g++) if (PIN_ILLUM[g] != NO_PIN || ILLUM_CHIPLED[g]) m |= 1 << g; return m; }
  void lampOffAll() { illum.off(); if (_drv && _chipLedsOn) { _drv->ledsOff(); _chipLedsOn = false; } }

  bool beginProc(Proc p, const char*& why) {
    _proc = p; _ok = false; _err = nullptr; _gIdx = 0; _nFr = 0;
    _useGroups = (p == P_WHITE) ? _nw.groups : (groups & _w.groups);
    memset(_rate, 0, sizeof _rate); memset(_rateSe, 0, sizeof _rateSe); memset(_satG, 0, sizeof _satG);
    _lampMs0 = illum.totalMs();
    if (!nextGroup()) { _proc = P_IDLE; why = "NO_LAMP_GROUP"; return false; }
    return true;
  }
  // advance to the next group in _useGroups; false when none left
  bool nextGroup() {
    while (_gIdx < N_ILLUM && !((_useGroups >> _gIdx) & 1)) _gIdx++;
    if (_gIdx >= N_ILLUM) return false;
    uint8_t g = _gIdx;
    if (_proc == P_WHITE) { _gain = _drv->gainStart(); _tint = _drv->tintQuant(REFLECT_TINT_START_MS); _gainMoved = false; }
    else                  { _gain = _w.gain[g]; _tint = _drv->tintQuant(_w.tint[g]); }
    _ae = true; _aeTry = 0; _i = 0;
    memset(_sum, 0, sizeof _sum); memset(_sum2, 0, sizeof _sum2); _nLit = 0; _satMask = 0;
    progress();
    beginFrame((uint8_t)(1 << g), _gain, _tint);
    return true;
  }
  void progress() {
    J.begin("progress").kvu("ms", millis()).kvs("proc", PROC_NAME[_proc]).kvs("group", ILLUM_NAME[_gIdx])
     .kvs("phase", _ae ? "AE" : "CAP").kvu("i", _i).kvu("n", 2u * frames + 1u).kvf("tint_ms", _tint, 1).kvf("gain", _drv->gainF(_gain), 1);
    emit();
  }
  // ---- one frame: lamp state, settle, then the chip ----
  void beginFrame(uint8_t lampMask, uint8_t gain, float tint) {
    const char* why = nullptr;
    _lit = lampMask != 0; _fGain = gain; _fTint = tint;
    uint8_t chipLeds = 0;
    if (_lit) {
      for (uint8_t g = 0; g < N_ILLUM; g++) if ((lampMask >> g) & 1) chipLeds |= ILLUM_CHIPLED[g];
      if (!illum.on(lampMask, REFLECT_LAMP_CAP_MS, why)) { abort(why); return; }
    } else illum.off();
    _chipLedsOn = chipLeds != 0;
    _fLeds = chipLeds; _settleAt = millis(); _started = false;
  }
  void run() {
    if (!_started) {
      if (since(_settleAt) < REFLECT_SETTLE_MS) return;
      _drv->start(_fGain, _fTint, _fLeds); _started = true; _fT0 = millis();
    }
    int8_t r = _drv->poll();
    if (r == 0) return;
    if (r < 0) { markFail(_drv->err); abort(_drv->err); return; }
    markOk(); _nFr++;
    if (_proc == P_LIVE) { liveFrame(); return; }
    onFrame();
  }
  float peakFrac() const {
    float fs = _drv->fullScale(_fTint), p = 0;
    for (uint8_t b = 0; b < _drv->nb; b++) { float f = _drv->raw[b] / fs; if (f > p) p = f; }
    return _drv->sat ? fmaxf(p, 1.0f) : p;
  }
  void liveFrame() {
    J.begin("frame").kvu("ms", millis()).kvs("chip", chipName()).kvs("mode", liveModeName())
     .kvu("gain_idx", _fGain).kvf("gain", _drv->gainF(_fGain), 1).kvf("tint_ms", _fTint, 1).arr("raw");
    for (uint8_t b = 0; b < _drv->nb; b++) J.ei(_drv->raw[b]);
    J.close(']').kvf("peak", peakFrac(), 3).kvb("sat", _drv->sat);
    emit(false);
    if (since(_t0) >= _liveMs) { lampOffAll(); _proc = P_IDLE; _ok = true; logEvent("info", "LIVE_DONE"); return; }
    beginFrame(_liveMask, manGain, manTint);
  }
  const char* liveModeName() const {
    if (!_liveMask) return "DARK";
    if (_liveMask == (1 << N_ILLUM) - 1 && N_ILLUM > 1) return "ALL";
    for (uint8_t g = 0; g < N_ILLUM; g++) if (_liveMask == (1 << g)) return ILLUM_NAME[g];
    return "MIX";
  }

  // ---- WHITE / SCAN frames ----
  void onFrame() {
    uint8_t g = _gIdx; const uint8_t gm = (uint8_t)(1 << g);
    if (_ae) {                                                        // auto-exposure: one lit frame per try
      float p = peakFrac(); _aeTry++;
      bool inWin = p >= REFLECT_AE_LO && p <= REFLECT_AE_HI;
      if (!inWin && _aeTry < REFLECT_AE_TRIES) {
        float t = _tint * (p >= 0.98f ? 0.25f : REFLECT_AE_TARGET / fmaxf(p, 0.005f));
        float tq = _drv->tintQuant(fminf(fmaxf(t, _drv->tintMin()), fminf(REFLECT_TINT_MAX_MS, _drv->tintMax())));
        uint8_t gNew = _gain;
        if (_proc == P_WHITE) {                                      // gain moves only in WHITE, and only when time can't
          if (p > REFLECT_AE_HI && tq <= _drv->tintMin() + 0.01f && _gain > 0) gNew = _gain - 1;
          if (p < REFLECT_AE_LO && tq >= fminf(REFLECT_TINT_MAX_MS, _drv->tintMax()) - 0.01f && _gain + 1 < _drv->nGain()) gNew = _gain + 1;
        }
        bool stuck = (tq == _tint && gNew == _gain);
        if (!stuck) {
          if (gNew != _gain) { _gainMoved = true; tq = _drv->tintQuant(REFLECT_TINT_START_MS); }
          _gain = gNew; _tint = tq;
          beginFrame(gm, _gain, _tint); return;
        }
      }
      if (_proc == P_WHITE && !inWin && p > REFLECT_AE_HI) { abort("WHITE_TOO_BRIGHT_LOWER_LED_CURRENT"); return; }
      if (_proc == P_WHITE && !inWin && p < REFLECT_AE_LO * 0.2f) { abort("WHITE_TOO_DARK_CHECK_LAMPS"); return; }
      _aePeak = p; _ae = false; _i = 0; progress();
      beginFrame(0, _gain, _tint);                                   // capture starts with a dark frame
      return;
    }
    // capture: D L D L ... L D  (frame i even = dark, odd = lit)
    const uint8_t nb = _drv->nb;
    float scale = 1.0f / (_tint * _drv->gainF(_gain));
    if ((_i & 1) == 0) {                                              // dark
      if (_i > 0) {                                                   // close the pending lit frame
        for (uint8_t b = 0; b < nb; b++) {
          float d = 0.5f * (_dPrev[b] + _drv->raw[b]);
          float r = (_lPend[b] - d) * scale;
          _sum[b] += r; _sum2[b] += r * r;
        }
        _nLit++;
      }
      for (uint8_t b = 0; b < nb; b++) _dPrev[b] = _drv->raw[b];
    } else {                                                          // lit
      float fs = _drv->fullScale(_tint);
      for (uint8_t b = 0; b < nb; b++) { _lPend[b] = _drv->raw[b]; if (_drv->raw[b] >= fs * 0.985f) _satMask |= 1ul << b; }
    }
    _i++;
    if (_i < 2u * frames + 1u) { beginFrame((_i & 1) ? gm : 0, _gain, _tint); return; }
    // group done: mean rate and its standard error per band
    for (uint8_t b = 0; b < nb; b++) {
      float m = _sum[b] / _nLit;
      float var = _nLit > 1 ? fmaxf(0.0f, (_sum2[b] - _nLit * m * m) / (_nLit - 1)) : 0.0f;
      float se = sqrtf(var / _nLit);
      float floor_ = 0.0005f * fabsf(m) + 0.7f * scale / sqrtf((float)_nLit);   // never claim better than ~0.05 % + read noise
      if (_nLit < 2) floor_ = fmaxf(floor_, 0.01f * fabsf(m));
      _rate[g][b] = m; _rateSe[g][b] = fmaxf(se, floor_);
    }
    _satG[g] = _satMask; _gainG[g] = _gain; _tintG[g] = _tint; _peakG[g] = _aePeak; _movedG[g] = _gainMoved;
    _gIdx++;
    if (nextGroup()) return;
    lampOffAll();
    if (_proc == P_WHITE) finishWhite(); else finishScan();
  }
  void finishWhite() {
    const uint8_t nb = _drv->nb;
    for (uint8_t g = 0; g < N_ILLUM; g++) {
      if (!((_nw.groups >> g) & 1)) continue;
      float mx = 0; for (uint8_t b = 0; b < nb; b++) if (_rate[g][b] > mx) mx = _rate[g][b];
      uint32_t v = 0;
      for (uint8_t b = 0; b < nb; b++) {
        _nw.W[g][b] = _rate[g][b]; _nw.sW[g][b] = _rateSe[g][b];
        bool lit = _rate[g][b] >= REFLECT_W_MIN_FRAC * mx && _rate[g][b] >= REFLECT_W_MIN_SNR * _rateSe[g][b];
        if (lit && !((_satG[g] >> b) & 1)) v |= 1ul << b;
      }
      _nw.valid[g] = v; _nw.gain[g] = _gainG[g]; _nw.tint[g] = _tintG[g]; _nw.peak[g] = _peakG[g]; _nw.gainMoved[g] = _movedG[g];
    }
    _w = _nw; _wFresh = true; _wAt = millis();
    Store::save(REFLECT_WHITE_STORE_OFF, &_w, sizeof _w, WHITE_MAGIC, WHITE_VERSION);
    _ok = true; _proc = P_IDLE; _t = millis();
    J.begin("result").kvs("kind", "white").kvu("ms", millis()).kvs("chip", chipName()).kvu("frames", frames).arr("wl");
    for (uint8_t b = 0; b < nb; b++) J.ei(_drv->wl[b]);
    J.close(']').arr("groups");
    for (uint8_t g = 0; g < N_ILLUM; g++) {
      if (!((_w.groups >> g) & 1)) continue;
      J.eobj().kvs("g", ILLUM_NAME[g]).kvu("gain_idx", _w.gain[g]).kvf("gain", _drv->gainF(_w.gain[g]), 1).kvf("tint_ms", _w.tint[g], 1)
       .kvf("peak", _w.peak[g], 3).kvb("gain_moved", _w.gainMoved[g]).kvu("valid", _w.valid[g]).kvu("sat", _satG[g]).arr("W");
      for (uint8_t b = 0; b < nb; b++) J.ef(_w.W[g][b], 3);
      J.close(']').close('}');
    }
    J.close(']').kvu("lamp_on_ms", illum.totalMs() - _lampMs0);
    emit();
  }
  void finishScan() {
    const uint8_t nb = _drv->nb;
    float R[RF::NB], sR[RF::NB]; int8_t grp[RF::NB]; uint32_t satMask = 0, nullMask = 0;
    for (uint8_t b = 0; b < nb; b++) {
      int8_t best = -1; float bestSnr = 0;
      for (uint8_t g = 0; g < N_ILLUM; g++) {
        if (!((_useGroups >> g) & 1) || !((_w.valid[g] >> b) & 1)) continue;
        float snr = _w.W[g][b] / fmaxf(_w.sW[g][b], 1e-9f);
        if (snr > bestSnr) { bestSnr = snr; best = (int8_t)g; }
      }
      grp[b] = best;
      if (best < 0) { R[b] = NAN; sR[b] = NAN; nullMask |= 1ul << b; continue; }
      if ((_satG[best] >> b) & 1) { R[b] = NAN; sR[b] = NAN; satMask |= 1ul << b; nullMask |= 1ul << b; continue; }
      float W = _w.W[best][b], S = _rate[best][b];
      R[b] = S / W;
      sR[b] = sqrtf(_rateSe[best][b] * _rateSe[best][b] + R[b] * R[b] * _w.sW[best][b] * _w.sW[best][b]) / W;
    }
    _ok = true; _proc = P_IDLE; _t = millis();
    J.begin("result").kvs("kind", "reflect").kvu("ms", millis()).kvs("tag", _tag).kvs("chip", chipName()).kvu("frames", frames).arr("wl");
    for (uint8_t b = 0; b < nb; b++) J.ei(_drv->wl[b]);
    J.close(']').arr("R");  for (uint8_t b = 0; b < nb; b++) J.ef(R[b], 4);
    J.close(']').arr("sR"); for (uint8_t b = 0; b < nb; b++) J.ef(sR[b], 4);
    J.close(']').arr("grp"); for (uint8_t b = 0; b < nb; b++) J.ei(grp[b]);
    J.close(']').kvu("sat_mask", satMask).kvu("null_mask", nullMask).arr("groups");
    for (uint8_t g = 0; g < N_ILLUM; g++) {
      if (!((_useGroups >> g) & 1)) continue;
      J.eobj().kvs("g", ILLUM_NAME[g]).kvu("gain_idx", _gainG[g]).kvf("tint_ms", _tintG[g], 1).kvf("peak", _peakG[g], 3).close('}');
    }
    bool moved = false; for (uint8_t g = 0; g < N_ILLUM; g++) moved |= ((_w.groups >> g) & 1) && _w.gainMoved[g];
    J.close(']').kv("white_age_s", whiteAgeS()).kvb("white_gain_moved", moved)
     .kvu("lamp_on_ms", illum.totalMs() - _lampMs0);
    emit();
  }
  void loadWhite() {
    WhiteRec w;
    if (Store::load(REFLECT_WHITE_STORE_OFF, &w, sizeof w, WHITE_MAGIC, WHITE_VERSION)) { _w = w; _wFresh = false; }
    else memset(&_w, 0, sizeof _w);
  }

  SpecDrv* _drv = nullptr; Proc _proc = P_IDLE; bool _ok = false; const char* _err = nullptr;
  uint32_t _t = 0, _t0 = 0, _settleAt = 0, _fT0 = 0, _liveMs = 0, _lampMs = 0, _lampMs0 = 0, _nFr = 0;
  uint8_t _liveMask = 0, _gIdx = 0, _useGroups = 0, _gain = 0, _fGain = 0, _fLeds = 0, _aeTry = 0, _nLit = 0;
  uint16_t _i = 0; float _tint = 0, _fTint = 0, _aePeak = 0; bool _ae = false, _lit = false, _started = false, _gainMoved = false, _chipLedsOn = false;
  uint32_t _satMask = 0;
  float _dPrev[RF::NB], _lPend[RF::NB], _sum[RF::NB], _sum2[RF::NB];
  float _rate[N_ILLUM][RF::NB], _rateSe[N_ILLUM][RF::NB], _tintG[N_ILLUM], _peakG[N_ILLUM];
  uint32_t _satG[N_ILLUM]; uint8_t _gainG[N_ILLUM]; bool _movedG[N_ILLUM];
  WhiteRec _w, _nw; bool _wFresh = false; uint32_t _wAt = 0;
  char _tag[24] = "";
};
constexpr const char* Reflect::PROC_NAME[5];
Reflect refl;
