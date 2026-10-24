#pragma once
// ============================================================================
//  sensors.h — THE BEDROCK: one class per sensor, all obeying the contract.
//  No sensor library is used: every driver talks registers through Bus::,
//  so a failed transfer is SEEN (libraries happily return garbage or hang)
//  and the core-3.0.0 NACK bug is handled in one place.
//    Imu6050      MPU6050      0x68  steady gate + tilt for the scales
//    Hx711Scale   HX711 x3     GPIO  rock / regolith / deep mass
//    Bme280       BME280       0x76  MET: T / RH / P (BMP280 impostor detected)
//    Scd4x        SCD40/41     0x62  analysis-chamber CO2 (carbonate test)
//    WindModbus   RS-485       UART  ultrasonic anemometer, Modbus RTU (TIM7 releases DE)
//    Ds18b20      1-Wire       PA15  LIQUID temperature for the pH slope
//    PhProbe      analog       PA1   Astro-Bio pH, temperature-corrected Nernst
//    LiquidSense  digital      PC2   optional: "is the intake tip in liquid?"
//  (Reflectance: reflect.h. SWIR: swir.h.)
// ============================================================================
#include "subsystem.h"
#include "actuators.h"

// ============================================================================
// IMU — MPU6050 raw registers. Tilt from gravity, motion from the gyro.
// Mount it Z up, X forward. Steady = quiet for STEADY_HOLD_MS and not too tilted.
// ============================================================================
class Imu6050 : public Subsystem {
 public:
  Imu6050() : Subsystem("imu", ENABLE_IMU, true) {}
  bool init() override {
    uint8_t who = 0;
    if (!Bus::readReg(ADDR_MPU, 0x75, &who, 1)) return initFail("NO_ACK");
    if (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x73) return initFail("BAD_WHO_AM_I");
    if (!Bus::writeReg(ADDR_MPU, 0x6B, 0x01) ||    // wake (it boots asleep), PLL clock
        !Bus::writeReg(ADDR_MPU, 0x19, 9)    ||    // 100 Hz sample rate
        !Bus::writeReg(ADDR_MPU, 0x1A, 3)    ||    // 44 Hz digital low-pass
        !Bus::writeReg(ADDR_MPU, 0x1B, 0)    ||    // +-250 dps
        !Bus::writeReg(ADDR_MPU, 0x1C, 0))         // +-2 g
      return initFail("CONFIG_WRITE");
    _biasN = 0; _bsx = _bsy = _bsz = 0; _quietSince = millis(); _t = millis();
    return true;
  }
  void poll() override {
    if (since(_t) < 20) return;                     // 50 Hz
    _t = millis();
    uint8_t b[14];
    if (!Bus::readReg(ADDR_MPU, 0x3B, b, 14)) { markFail("READ"); return; }
    auto s16 = [&](uint8_t i) { return (int16_t)(((uint16_t)b[i] << 8) | b[i + 1]); };
    float ax = s16(0) / 16384.0f, ay = s16(2) / 16384.0f, az = s16(4) / 16384.0f;
    float gx = s16(8) / 131.0f,   gy = s16(10) / 131.0f,  gz = s16(12) / 131.0f;
    float g = sqrtf(ax * ax + ay * ay + az * az);
    if (g < 0.3f || g > 3.0f) { markFail("IMPLAUSIBLE_G"); return; }
    _g = g;
    _pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    _roll  = atan2f(ay, az) * RAD_TO_DEG;
    _cosTilt = az / g;
    if (_biasN < 50) {                               // gyro bias: 1 s of stillness after (re)start
      if (fabsf(g - 1.0f) < STEADY_G_TOL && fabsf(gx) + fabsf(gy) + fabsf(gz) < 30.0f) {
        _bsx += gx; _bsy += gy; _bsz += gz; _biasN++;
        if (_biasN == 50) { _bx = _bsx / 50; _by = _bsy / 50; _bz = _bsz / 50; }
      } else { _biasN = 0; _bsx = _bsy = _bsz = 0; }
    }
    gx -= _bx; gy -= _by; gz -= _bz;
    _rate = fmaxf(fabsf(gx), fmaxf(fabsf(gy), fabsf(gz)));
    bool quiet = _biasN >= 50 && _rate < STEADY_RATE_DPS && fabsf(g - 1.0f) < STEADY_G_TOL;
    if (!quiet) _quietSince = millis();
    _steady = quiet && since(_quietSince) >= STEADY_HOLD_MS &&
              fabsf(_pitch) < STEADY_TILT_DEG && fabsf(_roll) < STEADY_TILT_DEG;
    markOk();
  }
  void rezero() { _biasN = 0; _bsx = _bsy = _bsz = 0; }
  bool  steady() const   { return healthy() && _steady; }
  float pitch() const    { return _pitch; }
  float roll() const     { return _roll; }
  float gMag() const     { return _g; }
  bool  biasOk() const   { return _biasN >= 50; }
  bool cosTilt(float& c) const {                   // cos(tilt) for the load cells; false if unknown or too steep
    if (!healthy() || _cosTilt < cosf(SCALE_TILT_MAX_DEG * DEG_TO_RAD)) return false;
    c = _cosTilt; return true;
  }
  void toJson(Json& j) override { j.kvf("pitch", _pitch, 1).kvf("roll", _roll, 1).kvf("rate", _rate, 1); }
 private:
  uint32_t _t = 0, _quietSince = 0; uint8_t _biasN = 0;
  float _bsx = 0, _bsy = 0, _bsz = 0, _bx = 0, _by = 0, _bz = 0;
  float _pitch = 0, _roll = 0, _rate = 0, _cosTilt = 1, _g = 1; bool _steady = false;
};
Imu6050 imu;

// The steady gate: the payload's own IMU AND the rover's motion state. If the
// IMU exists but is dead, we are NOT steady (fail safe). In ROS2 mode, stale
// rover-motion data counts as moving (state.h: roverMoving()).
bool systemSteady() {
  bool imuOk = imu.enabled ? imu.steady() : true;
  return imuOk && !roverMoving();
}

// ============================================================================
// HX711 LOAD CELL — 24-bit bridge ADC, 10 conversions/s, bit-banged.
// DOUT LOW = conversion ready. 24 clocks shift the data out, a 25th selects
// channel A gain 128 for the next one. SCK HIGH > 60 us = power-down, so each
// clock pulse is kept short with interrupts off for that 1 us only.
// ============================================================================
class Hx711Scale : public Subsystem {
 public:
  Hx711Scale(const char* key, uint8_t calIdx, uint32_t dout, uint32_t sck, bool en)
    : Subsystem(key, en, false), _ci(calIdx), _dout(dout), _sck(sck) {}
  bool init() override {
    pinMode(_sck, OUTPUT); digitalWrite(_sck, LOW);   // SCK low = awake
    pinMode(_dout, INPUT_PULLUP);                     // unplugged -> HIGH -> never "ready" -> fault
    _n = 0; _wi = 0; _lastConv = millis(); _job = JOB_NONE;
    return true;                                      // presence is proven by the first conversion
  }
  void poll() override {
    if (digitalRead(_dout) == LOW) {
      int32_t raw = readRaw();
      _lastConv = millis(); _seq++;
      if (raw == 0x7FFFFF || raw == -0x800000) { markFail("ADC_RAILED"); return; }   // open bridge / overload
      _win[_wi] = raw; _wi = (uint8_t)((_wi + 1) % SCALE_WINDOW); if (_n < SCALE_WINDOW) _n++;
      markOk();
      serviceJob(raw);
    } else if (since(_lastConv) > SCALE_TIMEOUT_MS) {
      _lastConv = millis(); markFail("NO_CONVERSION");
    }
  }
  void safe() override { _job = JOB_NONE; }

  // ---- readings ----
  bool calibrated() const { return cal.scaleCpg[_ci] != 0.0f; }
  bool tared() const      { return cal.taredMask & (1u << _ci); }   // no tare = offset 0 = a span would swallow the cell's zero
  int32_t rawMean() const { if (!_n) return 0; int64_t s = 0; for (uint8_t i = 0; i < _n; i++) s += _win[i]; return (int32_t)(s / _n); }
  float grams() const {                              // tilt-compensated net mass, NaN if unknown
    if (!healthy() || !calibrated() || _n == 0) return NAN;
    float g = (float)(rawMean() - cal.scaleOffset[_ci]) / cal.scaleCpg[_ci];
    float c; if (imu.cosTilt(c)) g /= c;            // the cell feels m*g*cos(tilt)
    return g;
  }
  bool tiltCompensated() const { float c; return imu.cosTilt(c); }
  bool stable() const {
    if (!healthy() || !calibrated() || _n < SCALE_WINDOW) return false;
    int32_t lo = _win[0], hi = _win[0];
    for (uint8_t i = 1; i < _n; i++) { lo = min(lo, _win[i]); hi = max(hi, _win[i]); }
    return fabsf((float)(hi - lo) / cal.scaleCpg[_ci]) <= SCALE_STABLE_G;
  }
  bool enough(float targetG) const { float g = grams(); return stable() && !isnan(g) && g >= targetG; }
  uint32_t seq() const { return _seq; }

  // ---- calibration jobs (collect 2 s of FRESH conversions, never block) ----
  bool startTare()           { if (!healthy()) return false; _job = JOB_TARE; jobReset(); return true; }
  bool startSpan(float refG) { if (!healthy() || !(refG > 0) || !tared()) return false; _job = JOB_SPAN; _jobRef = refG; jobReset(); return true; }
  bool jobBusy() const { return _job != JOB_NONE; }
  bool jobOk() const   { return _jobOk; }
  bool calibrate(const char* a, const char*& msg) override {        // "TARE" | "SPAN,<grams>"
    if (!strncmp(a, "TARE", 4))  { msg = startTare() ? "TARE_STARTED" : "SCALE_OFFLINE"; return _job != JOB_NONE; }
    if (!strncmp(a, "SPAN,", 5)) { msg = !tared() ? "TARE_FIRST" : startSpan(atof(a + 5)) ? "SPAN_STARTED" : "BAD_ARG_OR_OFFLINE"; return _job != JOB_NONE; }
    msg = "USE TARE | SPAN,<grams>"; return false;
  }
  void toJson(Json& j) override {
    char k[16];
    snprintf(k, sizeof k, "%s_g", name);  j.kvf(k, grams(), 1);
    snprintf(k, sizeof k, "%s_st", name); j.kvb(k, stable());
  }

 private:
  enum Job : uint8_t { JOB_NONE, JOB_TARE, JOB_SPAN };
  int32_t readRaw() {
    uint32_t v = 0;
    for (uint8_t i = 0; i < 24; i++) {
      noInterrupts(); digitalWrite(_sck, HIGH); delayMicroseconds(1);
      v = (v << 1) | (digitalRead(_dout) ? 1u : 0u);
      digitalWrite(_sck, LOW); interrupts(); delayMicroseconds(1);
    }
    noInterrupts(); digitalWrite(_sck, HIGH); delayMicroseconds(1); digitalWrite(_sck, LOW); interrupts();
    if (v & 0x800000u) v |= 0xFF000000u;             // sign-extend 24 -> 32 bits
    return (int32_t)v;
  }
  void jobReset() { _jobN = 0; _jobSum = 0; _jobMin = INT32_MAX; _jobMax = INT32_MIN; _jobOk = false; }
  void serviceJob(int32_t raw) {
    if (_job == JOB_NONE) return;
    _jobSum += raw; _jobMin = min(_jobMin, raw); _jobMax = max(_jobMax, raw);
    if (++_jobN < SCALE_WINDOW * 2) return;
    int32_t mean = (int32_t)(_jobSum / _jobN);
    float cpg = cal.scaleCpg[_ci];
    float spreadG = cpg != 0.0f ? fabsf((float)(_jobMax - _jobMin) / cpg) : 0.0f;
    const char* res = "OK";
    if (cpg != 0.0f && spreadG > 3.0f * SCALE_STABLE_G) res = "UNSTABLE_MOVE_LESS";
    else if (_job == JOB_TARE) { cal.scaleOffset[_ci] = mean; cal.taredMask |= (uint8_t)(1u << _ci); calSave(); }
    else {
      float c = (float)(mean - cal.scaleOffset[_ci]) / _jobRef;
      if (fabsf(c) < 1.0f) res = "NO_LOAD_CHANGE_TARE_FIRST";
      else { cal.scaleCpg[_ci] = c; calSave(); }
    }
    _jobOk = !strcmp(res, "OK");
    J.begin("cal").kvu("ms", millis()).kvs("sub", name).kvs("job", _job == JOB_TARE ? "TARE" : "SPAN")
     .kvb("ok", _jobOk).kvs("msg", res).kv("offset", cal.scaleOffset[_ci]).kvf("cpg", cal.scaleCpg[_ci], 3);
    emit();
    _job = JOB_NONE;
  }
  uint8_t _ci; uint32_t _dout, _sck;
  int32_t _win[SCALE_WINDOW]; uint8_t _n = 0, _wi = 0; uint32_t _lastConv = 0, _seq = 0;
  Job _job = JOB_NONE; uint16_t _jobN = 0; int64_t _jobSum = 0; int32_t _jobMin = 0, _jobMax = 0; float _jobRef = 0; bool _jobOk = false;
};
Hx711Scale scaleRock("rock", 0, PIN_HX_ROCK_DOUT, PIN_HX_ROCK_SCK, ENABLE_SCALES);
Hx711Scale scaleReg ("reg",  1, PIN_HX_REG_DOUT,  PIN_HX_REG_SCK,  ENABLE_SCALES);
Hx711Scale scaleDeep("deep", 2, PIN_HX_DEEP_DOUT, PIN_HX_DEEP_SCK, ENABLE_SCALES && ENABLE_DEEP_BAY);

// ============================================================================
// BME280 — Bosch integer compensation (datasheet sec. 4.2.3), normal mode,
// 1 s standby. Chip ID 0x60 = BME280; 0x58 = BMP280 sold as BME280 (no RH):
// accepted for T and P, humidity reported null and flagged.
// ============================================================================
class Bme280 : public Subsystem {
 public:
  Bme280() : Subsystem("met", ENABLE_MET, true) {}
  bool init() override {
    uint8_t id = 0;
    if (!Bus::readReg(ADDR_BME280, 0xD0, &id, 1)) return initFail("NO_ACK");
    if (id != 0x60 && id != 0x58) return initFail("UNKNOWN_CHIP_ID");
    _hasHum = (id == 0x60);
    uint8_t c[26];
    if (!Bus::readReg(ADDR_BME280, 0x88, c, 26)) return initFail("CAL_READ");
    auto u16 = [&](uint8_t i) { return (uint16_t)(c[i] | (c[i + 1] << 8)); };
    T1 = u16(0); T2 = (int16_t)u16(2); T3 = (int16_t)u16(4);
    P1 = u16(6); P2 = (int16_t)u16(8); P3 = (int16_t)u16(10); P4 = (int16_t)u16(12); P5 = (int16_t)u16(14);
    P6 = (int16_t)u16(16); P7 = (int16_t)u16(18); P8 = (int16_t)u16(20); P9 = (int16_t)u16(22);
    H1 = c[25];
    if (_hasHum) {
      uint8_t h[7];
      if (!Bus::readReg(ADDR_BME280, 0xE1, h, 7)) return initFail("CAL_READ_H");
      H2 = (int16_t)(h[0] | (h[1] << 8)); H3 = h[2];
      H4 = (int16_t)(((int8_t)h[3]) * 16 | (h[4] & 0x0F));
      H5 = (int16_t)(((int8_t)h[5]) * 16 | (h[4] >> 4));
      H6 = (int8_t)h[6];
      if (!Bus::writeReg(ADDR_BME280, 0xF2, 0x01)) return initFail("CONFIG_WRITE");   // RH x1 (before F4!)
    }
    if (!Bus::writeReg(ADDR_BME280, 0xF5, 0xA0) ||                                    // 1000 ms standby, no filter
        !Bus::writeReg(ADDR_BME280, 0xF4, 0x27)) return initFail("CONFIG_WRITE");     // T x1, P x1, normal mode
    _t = millis() - 900;
    return true;
  }
  void poll() override {
    if (since(_t) < 1000) return;
    _t = millis();
    uint8_t d[8];
    if (!Bus::readReg(ADDR_BME280, 0xF7, d, 8)) { markFail("READ"); return; }
    int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    int32_t adcH = ((int32_t)d[6] << 8) | d[7];
    if (adcT == 0x80000) return;                     // reset value: first conversion not done yet
    float t = compT(adcT) / 100.0f;
    float p = compP(adcP) / 25600.0f;                // Q24.8 Pa -> hPa
    float h = _hasHum ? compH(adcH) / 1024.0f : NAN;
    if (t < -40 || t > 85 || p < 300 || p > 1100 || (_hasHum && (h < 0 || h > 100))) { markFail("IMPLAUSIBLE"); return; }
    _tC = t; _hPa = p; _rh = h; markOk();
  }
  float tempC() const   { return healthy() ? _tC : NAN; }
  float hPa() const     { return healthy() ? _hPa : NAN; }
  float rh() const      { return healthy() ? _rh : NAN; }
  bool  hasHumidity() const { return _hasHum; }
  void toJson(Json& j) override { j.kvf("temp", tempC(), 2).kvf("hum", rh(), 1).kvf("pres", hPa(), 2); }
 private:
  int32_t compT(int32_t adc) {
    int32_t v1 = ((((adc >> 3) - ((int32_t)T1 << 1))) * (int32_t)T2) >> 11;
    int32_t v2 = (((((adc >> 4) - (int32_t)T1) * ((adc >> 4) - (int32_t)T1)) >> 12) * (int32_t)T3) >> 14;
    tFine = v1 + v2;
    return (tFine * 5 + 128) >> 8;
  }
  uint32_t compP(int32_t adc) {
    int64_t v1 = (int64_t)tFine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)P6;
    v2 = v2 + ((v1 * (int64_t)P5) << 17);
    v2 = v2 + ((int64_t)P4 << 35);
    v1 = ((v1 * v1 * (int64_t)P3) >> 8) + ((v1 * (int64_t)P2) << 12);
    v1 = ((((int64_t)1) << 47) + v1) * (int64_t)P1 >> 33;
    if (v1 == 0) return 0;
    int64_t p = 1048576 - adc;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)P8 * p) >> 19;
    p = ((p + v1 + v2) >> 8) + ((int64_t)P7 << 4);
    return (uint32_t)p;
  }
  uint32_t compH(int32_t adc) {
    int32_t v = tFine - 76800;
    v = (((((adc << 14) - ((int32_t)H4 << 20) - ((int32_t)H5 * v)) + 16384) >> 15) *
         (((((((v * (int32_t)H6) >> 10) * (((v * (int32_t)H3) >> 11) + 32768)) >> 10) + 2097152) * (int32_t)H2 + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)H1) >> 4);
    v = v < 0 ? 0 : v; v = v > 419430400 ? 419430400 : v;
    return (uint32_t)(v >> 12);
  }
  uint16_t T1 = 0, P1 = 0; int16_t T2 = 0, T3 = 0, P2 = 0, P3 = 0, P4 = 0, P5 = 0, P6 = 0, P7 = 0, P8 = 0, P9 = 0;
  uint8_t H1 = 0, H3 = 0; int16_t H2 = 0, H4 = 0, H5 = 0; int8_t H6 = 0; int32_t tFine = 0;
  bool _hasHum = true; uint32_t _t = 0; float _tC = NAN, _hPa = NAN, _rh = NAN;
};
Bme280 met;

// ============================================================================
// SCD4x — Sensirion command protocol: 16-bit commands, every 16-bit word
// answered with a CRC-8 (poly 0x31, init 0xFF). Periodic mode = one sample
// every 5 s. Two-phase reads (send, wait >= 1 ms, read) so nothing blocks.
// We never send wake_up: the SCD4x doesn't ACK it, and that NACK kills the
// core-3.0.0 I2C driver.
// ============================================================================
static uint8_t sensirionCrc(uint8_t a, uint8_t b) {
  uint8_t crc = 0xFF; uint8_t d[2] = { a, b };
  for (uint8_t i = 0; i < 2; i++) { crc ^= d[i]; for (uint8_t k = 0; k < 8; k++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1); }
  return crc;
}
class Scd4x : public Subsystem {
 public:
  Scd4x() : Subsystem("co2", ENABLE_CO2, true) {}
  bool init() override {
    if (!cmd(0x3F86)) return initFail("NO_ACK");      // stop_periodic: it may still be running from before our reset
    _ph = P_STOPPING; _t = millis(); _lastData = millis();
    return true;
  }
  void poll() override {
    switch (_ph) {
      case P_STOPPING:                                // datasheet: 500 ms before the next command
        if (since(_t) < 500) return;
        if (!cmd(0x21B1)) { markFail("START"); _t = millis(); return; }
        _ph = P_RUN; _t = millis(); _lastData = millis(); _tPress = millis();
        return;
      case P_RUN:
        if (since(_lastData) > 12000) { _lastData = millis(); markFail("STALE_NO_NEW_DATA"); }
        if (_pressHpa > 0 && since(_tPress) > 60000) {   // ambient-pressure compensation from the BME280
          _tPress = millis();
          uint16_t hp = (uint16_t)(_pressHpa + 0.5f);
          uint8_t b[5] = { 0xE0, 0x00, (uint8_t)(hp >> 8), (uint8_t)hp, sensirionCrc((uint8_t)(hp >> 8), (uint8_t)hp) };
          if (!Bus::write(ADDR_SCD4X, b, 5)) markFail("SET_PRESSURE");
          return;
        }
        if (since(_t) < 250) return;
        _t = millis();
        if (!cmd(0xE4B8)) { markFail("READY_CMD"); return; }
        _ph = P_READY_SENT; _tUs = micros();
        return;
      case P_READY_SENT: {
        if (sinceUs(_tUs) < 1200) return;
        uint16_t w;
        if (!readWords(&w, 1)) { _ph = P_RUN; return; }
        markOk();                                     // CRC-valid answer = the sensor is alive
        if ((w & 0x07FF) == 0) { _ph = P_RUN; return; }
        if (!cmd(0xEC05)) { markFail("MEAS_CMD"); _ph = P_RUN; return; }
        _ph = P_MEAS_SENT; _tUs = micros();
        return;
      }
      case P_MEAS_SENT: {
        if (sinceUs(_tUs) < 1200) return;
        uint16_t w[3];
        _ph = P_RUN;
        if (!readWords(w, 3)) return;
        if (w[0] == 0) return;                        // "no data" pattern
        _co2 = w[0]; _tC = -45.0f + 175.0f * w[1] / 65535.0f; _rh = 100.0f * w[2] / 65535.0f;
        _seq++; _lastData = millis(); markOk();
        return;
      }
    }
  }
  void setAmbientPressure(float hPa) { _pressHpa = (hPa > 300 && hPa < 1100) ? hPa : 0; }
  int32_t  co2() const   { return healthy() && _seq ? (int32_t)_co2 : -1; }
  uint32_t seq() const   { return _seq; }
  void toJson(Json& j) override { if (co2() >= 0) j.kv("co2", co2()); else j.kvf("co2", NAN, 0); }   // NaN -> null
 private:
  enum Ph : uint8_t { P_STOPPING, P_RUN, P_READY_SENT, P_MEAS_SENT };
  bool cmd(uint16_t c) { uint8_t b[2] = { (uint8_t)(c >> 8), (uint8_t)c }; return Bus::write(ADDR_SCD4X, b, 2); }
  bool readWords(uint16_t* w, uint8_t n) {
    uint8_t b[9];
    if (!Bus::read(ADDR_SCD4X, b, (uint8_t)(n * 3))) { markFail("READ"); return false; }
    for (uint8_t i = 0; i < n; i++) {
      if (sensirionCrc(b[3 * i], b[3 * i + 1]) != b[3 * i + 2]) { markFail("CRC"); return false; }
      w[i] = (uint16_t)((b[3 * i] << 8) | b[3 * i + 1]);
    }
    return true;
  }
  Ph _ph = P_STOPPING; uint32_t _t = 0, _tUs = 0, _lastData = 0, _tPress = 0, _seq = 0;
  uint16_t _co2 = 0; float _tC = NAN, _rh = NAN, _pressHpa = 0;
};
Scd4x co2;

// ============================================================================
// WIND — Modbus RTU master on RS-485 (USART1, half duplex, DE/RE on PB5).
// Request: [addr][0x03][reg hi][reg lo][count hi][count lo][crc lo][crc hi]
// DE must drop right after the last stop bit: the sensor answers ~3.5 chars
// later. 2.0 released it from the loop and was up to ~4 ms late (the loop
// was busy elsewhere). 3.0 arms a ONE-SHOT hardware timer (TIM7) for the
// frame's air time; its interrupt drops DE on time, whatever the loop does.
// ============================================================================
Uart Rs485(PIN_485_RX, PIN_485_TX);              // STM32duino's concrete UART class (HardwareSerial is the abstract API)
static uint16_t modbusCrc(const uint8_t* p, uint8_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) { crc ^= *p++; for (uint8_t k = 0; k < 8; k++) crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1); }
  return crc;
}
HardwareTimer* g_deTimer = nullptr;
volatile bool     g_deDropped = false;
volatile uint32_t g_deArmUs = 0;
constexpr uint32_t DE_AIR_US = (uint32_t)((8UL * 10UL + 5UL) * 1000000UL / WIND_BAUD);   // 8 chars of 10 bits + half a char margin
void deDropIsr() {
  // Starting a timer leaves an "update" event pending (STM32: UG sets UIF), which can fire the
  // interrupt the instant it is enabled. That one is not ours: ignore anything well before the air time.
  if ((uint32_t)(micros() - g_deArmUs) + 300 < DE_AIR_US) return;
  digitalWrite(PIN_485_DE, LOW); g_deDropped = true; g_deTimer->pause();
}

class WindModbus : public Subsystem {
 public:
  WindModbus() : Subsystem("wind", ENABLE_WIND, false) {}
  bool init() override {
    pinOutLow(PIN_485_DE);                            // receive mode
    if (!g_deTimer) { g_deTimer = new HardwareTimer(TIM7); g_deTimer->setOverflow(DE_AIR_US, MICROSEC_FORMAT); g_deTimer->attachInterrupt(deDropIsr); }
    g_deTimer->pause();
    Rs485.begin(WIND_BAUD);
    while (Rs485.available()) Rs485.read();
    _ph = W_IDLE; _t = millis() - WIND_POLL_MS;
    return true;                                      // presence = first valid reply
  }
  void poll() override {
    switch (_ph) {
      case W_IDLE: {
        if (since(_t) < WIND_POLL_MS) return;
        _t = millis();
        while (Rs485.available()) Rs485.read();
        uint8_t f[8] = { WIND_ADDR, 0x03, (uint8_t)(WIND_REG_START >> 8), (uint8_t)WIND_REG_START, 0, WIND_REG_COUNT, 0, 0 };
        uint16_t c = modbusCrc(f, 6); f[6] = (uint8_t)c; f[7] = (uint8_t)(c >> 8);
        g_deDropped = false;
        digitalWrite(PIN_485_DE, HIGH);
        Rs485.write(f, 8);                            // queued: the UART interrupt sends it, ~8.3 ms on the wire
        g_deArmUs = micros();
        g_deTimer->setCount(0);
        __HAL_TIM_CLEAR_FLAG(g_deTimer->getHandle(), TIM_FLAG_UPDATE);
        g_deTimer->resume();                          // one-shot: the ISR pauses it again
        _txUs = g_deArmUs;
        _ph = W_TX; return;
      }
      case W_TX:
        if (!g_deDropped && sinceUs(_txUs) < DE_AIR_US + 5000) return;   // the timer drops DE; 5 ms backstop
        if (!g_deDropped) { digitalWrite(PIN_485_DE, LOW); g_deTimer->pause(); _lateDe++; }
        _n = 0; _tRx = millis(); _ph = W_RX; return;
      case W_RX: {
        while (Rs485.available() && _n < sizeof(_rx)) _rx[_n++] = (uint8_t)Rs485.read();
        uint8_t expect = (_n >= 2 && (_rx[1] & 0x80)) ? 5 : (uint8_t)(5 + (_n >= 3 ? _rx[2] : 255));
        if (_n >= 3 && _n >= expect) { parse(expect); _ph = W_IDLE; return; }
        if (since(_tRx) > WIND_REPLY_TIMEOUT_MS) { markFail(_n ? "SHORT_REPLY" : "NO_REPLY"); _ph = W_IDLE; }
        return;
      }
    }
  }
  void safe() override { if (g_deTimer) g_deTimer->pause(); digitalWrite(PIN_485_DE, LOW); }
  float speed() const { return healthy() ? _speed : NAN; }
  float dir() const   { return healthy() ? _dir : NAN; }
  uint32_t lateDe() const { return _lateDe; }
  void toJson(Json& j) override { j.kvf("wind_s", speed(), 2).kvf("wind_d", dir(), 0); }
 private:
  void parse(uint8_t len) {
    uint16_t c = modbusCrc(_rx, (uint8_t)(len - 2));
    if (_rx[len - 2] != (uint8_t)c || _rx[len - 1] != (uint8_t)(c >> 8)) { markFail("CRC"); return; }
    if (_rx[0] != WIND_ADDR)                  { markFail("WRONG_ADDR"); return; }
    if (_rx[1] & 0x80)                        { markFail("MODBUS_EXCEPTION"); return; }
    if (_rx[2] != 2 * WIND_REG_COUNT)         { markFail("BYTE_COUNT"); return; }
    auto reg = [&](uint8_t i) { return (uint16_t)((_rx[3 + 2 * i] << 8) | _rx[4 + 2 * i]); };
    _speed = reg(WIND_SPEED_IDX) * WIND_SPEED_SCALE;
    _dir   = reg(WIND_DIR_IDX)   * WIND_DIR_SCALE;
    markOk();
  }
  enum Ph : uint8_t { W_IDLE, W_TX, W_RX };
  Ph _ph = W_IDLE; uint32_t _t = 0, _tRx = 0, _txUs = 0, _lateDe = 0;
  uint8_t _rx[32]; uint8_t _n = 0; float _speed = NAN, _dir = NAN;
};
WindModbus wind;

// ============================================================================
// DS18B20 — 1-Wire, bit-banged on an open-drain pin with a 4.7 k pull-up.
// The bus is a single wire: the master pulls it LOW for a timed slot, and
// the LENGTH of the low pulse is the message (6 us = 1, 60 us = 0; a slave
// answers a read slot by holding the line low). Only each ~70 us slot runs
// with interrupts off. One conversion = 750 ms, waited for without blocking;
// the 9-byte read is split over three loop passes.
// ============================================================================
static uint8_t crc8Dallas(const uint8_t* p, uint8_t n) {
  uint8_t c = 0;
  while (n--) { uint8_t b = *p++; for (uint8_t k = 0; k < 8; k++) { uint8_t mix = (c ^ b) & 1; c >>= 1; if (mix) c ^= 0x8C; b >>= 1; } }
  return c;
}
class Ds18b20 : public Subsystem {
 public:
  Ds18b20() : Subsystem("pht", ENABLE_PH_TEMP, false) {}
  bool init() override {
    pinMode(PIN_ONEWIRE, OUTPUT_OPEN_DRAIN); digitalWrite(PIN_ONEWIRE, HIGH);   // released: the pull-up holds it high
    _st = S_IDLE; _t = millis() - ONEWIRE_PERIOD_MS;
    return true;                                      // presence is proven by the first good reading
  }
  void poll() override {
    switch (_st) {
      case S_IDLE:
        if (since(_t) < ONEWIRE_PERIOD_MS) return;
        _t = millis();
        if (!reset()) { markFail("NO_PRESENCE"); return; }
        writeByte(0xCC); writeByte(0x44);             // SKIP ROM, CONVERT T
        _st = S_CONVERTING; return;
      case S_CONVERTING:
        if (since(_t) < 760) return;                  // 12-bit conversion: 750 ms max
        if (!reset()) { markFail("NO_PRESENCE"); _st = S_IDLE; return; }
        writeByte(0xCC); writeByte(0xBE);             // SKIP ROM, READ SCRATCHPAD
        _k = 0; _st = S_READ; return;
      case S_READ:
        for (uint8_t i = 0; i < 3 && _k < 9; i++) _sp[_k++] = readByte();   // 3 bytes per loop pass (~1.7 ms)
        if (_k < 9) return;
        _st = S_IDLE;
        if (crc8Dallas(_sp, 8) != _sp[8]) { markFail("CRC"); return; }
        {
          int16_t raw = (int16_t)((_sp[1] << 8) | _sp[0]);
          float c = raw / 16.0f;
          if (raw == 0x0550) { markFail("NOT_CONVERTED_85C"); return; }   // power-on value: the convert never ran
          if (c < -20.0f || c > 80.0f) { markFail("IMPLAUSIBLE"); return; }
          _tC = c; _seq++; markOk();
        }
        return;
    }
  }
  float tempC() const { return healthy() && _seq ? _tC : NAN; }
  void toJson(Json& j) override { j.kvf("liq_t", tempC(), 2); }
 private:
  enum St : uint8_t { S_IDLE, S_CONVERTING, S_READ };
  bool reset() {                                      // 480 us low, release, sample presence at +70 us
    digitalWrite(PIN_ONEWIRE, LOW); delayMicroseconds(480);
    noInterrupts();
    digitalWrite(PIN_ONEWIRE, HIGH); delayMicroseconds(70);
    bool present = digitalRead(PIN_ONEWIRE) == LOW;
    interrupts();
    delayMicroseconds(410);
    return present;
  }
  void writeBit(bool b) {
    noInterrupts();
    digitalWrite(PIN_ONEWIRE, LOW); delayMicroseconds(b ? 6 : 60);
    digitalWrite(PIN_ONEWIRE, HIGH); delayMicroseconds(b ? 64 : 10);
    interrupts();
  }
  bool readBit() {
    noInterrupts();
    digitalWrite(PIN_ONEWIRE, LOW); delayMicroseconds(6);
    digitalWrite(PIN_ONEWIRE, HIGH); delayMicroseconds(9);
    bool b = digitalRead(PIN_ONEWIRE) == HIGH;
    interrupts();
    delayMicroseconds(55);
    return b;
  }
  void writeByte(uint8_t v) { for (uint8_t i = 0; i < 8; i++) writeBit((v >> i) & 1); }     // LSB first
  uint8_t readByte() { uint8_t v = 0; for (uint8_t i = 0; i < 8; i++) if (readBit()) v |= (uint8_t)(1 << i); return v; }
  St _st = S_IDLE; uint32_t _t = 0, _seq = 0; uint8_t _sp[9]; uint8_t _k = 0; float _tC = NAN;
};
Ds18b20 liqTemp;

// ============================================================================
// pH — analog module through a 10k/20k divider. The electrode is a tiny
// battery: E = E0 - (2.303RT/F)*pH, so the slope scales with ABSOLUTE
// temperature. Two-point calibration (e.g. pH 7.00 and 4.01) fixes offset and
// slope at Tcal; at any other T the slope is rescaled by T/Tcal (isopotential
// point assumed at pH 7). A pH without its temperature is half a measurement,
// so the temperature comes from the LIQUID (DS18B20) first, then the air
// (BME280), then 25 C, and the frame says which one was used.
// ============================================================================
class PhProbe : public Subsystem {
 public:
  PhProbe() : Subsystem("ph", ENABLE_PH, false) {}
  bool init() override { pinMode(PIN_PH, INPUT_ANALOG); _n = 0; _wi = 0; _stableSince = millis(); _t = millis(); return true; }
  void poll() override {
    if (since(_t) < 100) return;                     // 10 Hz
    _t = millis();
    uint32_t acc = 0;
    for (uint8_t i = 0; i < 16; i++) acc += analogRead(PIN_PH);          // 16x oversampling
    _v = (acc / 16.0f) * VDDA / 4095.0f * ANALOG_DIVIDER;               // module output volts
    if (_v < 0.05f || _v > 4.95f) { markFail("SIGNAL_OUT_OF_RANGE"); return; }
    float tl = liqTemp.tempC(), ta = met.tempC();
    if (!isnan(tl))      { _tC = tl; _src = "liquid"; }
    else if (!isnan(ta)) { _tC = ta; _src = "air"; }
    else                 { _tC = PH_FALLBACK_TEMP_C; _src = "default"; }
    float ph = phFromVolts(_v, _tC);
    if (isnan(ph) || ph < -1.0f || ph > 15.0f) { markFail("PH_IMPLAUSIBLE"); return; }
    _win[_wi] = ph; _vwin[_wi] = _v; _wi = (uint8_t)((_wi + 1) % PH_WINDOW); if (_n < PH_WINDOW) _n++;
    // Settled = the MEDIAN stops drifting (an electrode settles like a slow exponential).
    float m = median(_win);
    if (_n < PH_WINDOW || fabsf(m - _ref) > PH_STABLE_DRIFT) { _ref = m; _stableSince = millis(); }
    markOk();
  }
  float ph() const        { return healthy() && _n ? median(_win) : NAN; }
  float volts() const     { return healthy() && _n ? median(_vwin) : NAN; }
  float tempC() const     { return _tC; }
  const char* tempSrc() const { return _src; }
  bool  stable() const    { return healthy() && _n >= PH_WINDOW && since(_stableSince) >= PH_STABLE_HOLD_MS; }
  bool  calibrated() const{ return cal.phN >= 2; }
  float slopeMvPerPh() const { return 1000.0f * (cal.phV[1] - cal.phV[0]) / (cal.phP[1] - cal.phP[0]); }
  // "CMD:PH_CAL=<buffer pH>": captures the CURRENT stable reading as a calibration point.
  bool calibrate(const char* a, const char*& msg) override {
    if (!strcmp(a, "CLEAR")) { calDefaultsPh(); calSave(); msg = "PH_CAL_CLEARED"; return true; }
    float p = atof(a);
    if (!(p > 0.0f && p < 14.0f)) { msg = "BAD_PH"; return false; }
    if (!stable())                { msg = "NOT_STABLE_WAIT"; return false; }
    float v = volts();
    if (cal.phN >= 2) {                              // replace the point with the closest pH
      uint8_t k = fabsf(cal.phP[0] - p) < fabsf(cal.phP[1] - p) ? 0 : 1;
      cal.phP[k] = p; cal.phV[k] = v;
    } else if (cal.phN == 1) {
      if (fabsf(cal.phP[0] - p) < 1.5f) { cal.phP[0] = p; cal.phV[0] = v; msg = "POINT1_REPLACED_NEED_A_SECOND_BUFFER"; calSave(); return true; }
      cal.phP[1] = p; cal.phV[1] = v; cal.phN = 2;
    } else { cal.phP[0] = p; cal.phV[0] = v; cal.phN = 1; }
    if (cal.phN >= 2) {
      if (fabsf(cal.phV[1] - cal.phV[0]) / fabsf(cal.phP[1] - cal.phP[0]) < 0.02f) { calDefaultsPh(); calSave(); msg = "SLOPE_TOO_FLAT_PROBE_DEAD"; return false; }
      cal.phTcal = _tC;
    }
    calSave(); msg = cal.phN >= 2 ? "PH_CALIBRATED" : "POINT1_SAVED_NEED_A_SECOND_BUFFER";
    return true;
  }
  void toJson(Json& j) override { j.kvf("ph", ph(), 2).kvb("ph_st", stable()); }
 private:
  static float phFromVolts(float v, float tC) {
    float sCal = (cal.phV[1] - cal.phV[0]) / (cal.phP[1] - cal.phP[0]);   // volts per pH at Tcal
    if (fabsf(sCal) < 0.02f) return NAN;
    float s = sCal * (tC + 273.15f) / (cal.phTcal + 273.15f);
    float vIso = cal.phV[0] + sCal * (7.0f - cal.phP[0]);
    return 7.0f + (v - vIso) / s;
  }
  float median(const float* w) const {
    float t[PH_WINDOW]; for (uint8_t i = 0; i < _n; i++) t[i] = w[i];
    for (uint8_t i = 1; i < _n; i++) { float x = t[i]; int8_t j = i - 1; while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; } t[j + 1] = x; }
    return _n & 1 ? t[_n / 2] : 0.5f * (t[_n / 2 - 1] + t[_n / 2]);
  }
  float _win[PH_WINDOW], _vwin[PH_WINDOW]; uint8_t _n = 0, _wi = 0;
  uint32_t _t = 0, _stableSince = 0; float _v = NAN, _tC = PH_FALLBACK_TEMP_C, _ref = NAN; const char* _src = "default";
};
PhProbe phProbe;

// ============================================================================
// LIQUID SENSE (optional) — two bare electrodes at the intake tip: one to GND,
// one to PC2 with the internal pull-up (~40 k). Natural water bridges them
// (a few k) and pulls the pin LOW. Distilled water may not: that is fine, the
// Astro-Bio sample is natural water. Held for LIQUID_HOLD_MS to ignore splashes.
// ============================================================================
class LiquidSense : public Subsystem {
 public:
  LiquidSense() : Subsystem("liq", ENABLE_LIQUID_SENSE, false) {}
  bool init() override { pinMode(PIN_LIQUID, INPUT_PULLUP); _since = millis(); _raw = false; _wet = false; return true; }
  void poll() override {
    bool r = digitalRead(PIN_LIQUID) == LOW;
    if (r != _raw) { _raw = r; _since = millis(); }
    if (since(_since) >= LIQUID_HOLD_MS) _wet = _raw;
    markOk();
  }
  bool wet() const { return healthy() && _wet; }
  void toJson(Json& j) override { j.kvb("liq", wet()); }
 private:
  bool _raw = false, _wet = false; uint32_t _since = 0;
};
LiquidSense liquid;
