#pragma once
// ============================================================================
//  subsystem.h — THE DESIGN LAW, as code. (shared by flight 3.0 and the bench)
//  "Sensors are the deterministic bedrock. Mechanics are quicksand."
//
//  Every sensor obeys ONE contract:
//     init()      bring the hardware up (also used to re-init after a fault)
//     poll()      do a little non-blocking work; call markOk()/markFail()
//     healthy()   true while the data can be trusted
//     recover()   operator "kick": retry init right now (CMD:RESET_<name>)
//     toJson()    this sensor's fields for the fast telemetry frame
//     calibrate() parse a calibration argument (CMD:CAL=<name>,<args>)
//
//  The base class owns the health state machine, so a dead sensor is skipped,
//  retried with back-off, and announced ONCE, without any sensor code knowing.
//
//      STARTING --first good data--> OK <--markOk-- DEGRADED
//         |  grace expires              |  N fails in a row    ^ 1 fail
//         v                             v                      |
//      OFFLINE <-----------------------------------------------
//         |  back-off 1-2-4-8 s: Wire.begin() + init()
//         +--> STARTING
// ============================================================================
#include "core.h"

enum Health : uint8_t { H_DISABLED, H_OFFLINE, H_STARTING, H_DEGRADED, H_OK };
const char* const HEALTH_NAME[] = { "dis", "off", "start", "deg", "ok" };

class Subsystem {
 public:
  const char* name;            // short telemetry/health key, also used by CMD:RESET_<name>
  const bool  enabled;
  const bool  onI2C;           // lives on the shared I2C bus (bus recovery applies)
  Subsystem(const char* n, bool en, bool i2c) : name(n), enabled(en), onI2C(i2c) {}
  virtual ~Subsystem() {}

  // ---- the contract ----
  virtual bool init() = 0;
  virtual void poll() = 0;
  virtual void toJson(Json&) {}
  virtual bool calibrate(const char* /*args*/, const char*& msg) { msg = "NO_CALIBRATION"; return false; }
  virtual void safe() {}                                   // stop anything physical this module drives
  bool healthy() const { return _h == H_OK || _h == H_DEGRADED; }
  void recover()       { _retryAt = millis(); _backoff = HEALTH_RETRY_MIN_MS; if (_h != H_DISABLED) _h = H_OFFLINE; }

  // ---- run by the registry every loop pass ----
  Health health() const      { return _h; }
  const char* lastError() const { return _err; }
  uint32_t errorCount() const   { return _errCount; }
  void service() {
    if (!enabled) { _h = H_DISABLED; return; }
    if (_h == H_OFFLINE) {
      if (!reached(_retryAt)) return;
      if (onI2C) Bus::prep();
      if (init()) { _h = H_STARTING; _startedAt = millis(); _fails = 0; }
      else {
        if (!_announcedUp && !_reportedMissing) { _reportedMissing = true; report("missing"); }
        scheduleRetry(); return;
      }
    }
    if (_h == H_STARTING && since(_startedAt) > HEALTH_STARTUP_MS) { goOffline("NO_DATA_AFTER_INIT"); return; }
    poll();
  }

 protected:
  void markOk() {
    _fails = 0;
    if (_h == H_STARTING || _h == H_DEGRADED) {
      bool announce = (_h == H_STARTING && !_announcedUp);
      _h = H_OK; _backoff = HEALTH_RETRY_MIN_MS;
      if (announce) { _announcedUp = true; _reportedMissing = false; _err = ""; report("up"); }
    }
  }
  bool initFail(const char* why) { _err = why; _errCount++; return false; }   // use in init(): return initFail("NO_ACK");
  void markFail(const char* why) {
    _err = why; _errCount++;
    if (_h == H_STARTING) return;                         // grace period: the deadline decides
    if (++_fails >= HEALTH_FAIL_LIMIT) goOffline(why);
    else if (_h == H_OK) _h = H_DEGRADED;
  }

 private:
  void goOffline(const char* why) {
    _err = why;
    if (_announcedUp)          report("down");      // was working, now lost
    else if (!_reportedMissing) report("missing"); // never came up: say so once
    _announcedUp = false; _reportedMissing = true;
    _h = H_OFFLINE; safe(); scheduleRetry();
    if (onI2C) { if (Bus::sdaStuckLow()) Bus::recover(); else Bus::dirty = true; }
  }
  void scheduleRetry() {
    _retryAt = millis() + _backoff;
    _backoff = _backoff * 2 > HEALTH_RETRY_MAX_MS ? HEALTH_RETRY_MAX_MS : _backoff * 2;
  }
  void report(const char* what) {
    J.begin("health_change").kvu("ms", millis()).kvs("sub", name).kvs("to", what).kvs("err", _err ? _err : "");
    emit();
  }
  Health   _h = H_OFFLINE;          // first service() call runs init() immediately
  uint32_t _retryAt = 0, _startedAt = 0, _backoff = HEALTH_RETRY_MIN_MS, _errCount = 0;
  uint8_t  _fails = 0;
  bool     _announcedUp = false, _reportedMissing = false;
  const char* _err = "";
};
