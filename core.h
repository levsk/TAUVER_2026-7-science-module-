#pragma once
// ============================================================================
//  core.h — the plumbing every TAUVER sketch shares (flight 3.0 AND the bench)
//    · time helpers (wrap-safe)          · JSON writer (no String, no heap)
//    · TX queue (Serial never blocks)    · RX line reader (never blocks)
//    · I2C bus manager (core-bug proof)  · Store: CRC-checked records in flash
//    · reset cause
//  It knows nothing about missions or sensors. Each sketch's config.h supplies
//  the constants used here (TX_RING_BYTES, RX_LINE_MAX, PIN_I2C_*, ...).
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"

// ----------------------------------------------------------------------------
// Time. Always measure "now - start" with unsigned math: it survives the
// 49-day millis() wrap. since() reads the clock itself, so a start time can
// never be "in the future" (the old Boss-2 race).
// ----------------------------------------------------------------------------
static inline uint32_t since(uint32_t t0)         { return millis() - t0; }
static inline uint32_t sinceUs(uint32_t t0)       { int32_t d = (int32_t)(micros() - t0); return d < 0 ? 0u : (uint32_t)d; }
// (micros() on STM32 is SysTick-derived and can step back ~1 ms in the instant
//  the tick wraps; a backwards step must DELAY us, never fire early)
static inline bool     reached(uint32_t deadline) { return (int32_t)(millis() - deadline) >= 0; }

static inline void pinOutLow(uint32_t p)          { if (p == NO_PIN) return; pinMode(p, OUTPUT); digitalWrite(p, LOW); }
static inline void pinSet(uint32_t p, bool on)    { if (p != NO_PIN) digitalWrite(p, on ? HIGH : LOW); }
// PWM pins are ALWAYS driven with analogWrite (0 = steady LOW, 255 = steady HIGH).
// Never mix in digitalWrite: once a pin is a timer output, digitalWrite can't touch it.
static inline void pwmSet(uint32_t p, uint8_t d)  { if (p != NO_PIN) analogWrite(p, d); }

// ----------------------------------------------------------------------------
// JSON writer: fills a fixed buffer, handles commas and nesting, escapes text,
// prints floats without printf (so the "Float Printf" runtime option can't
// silently break telemetry). NaN / inf -> null. Overflow = the frame is
// dropped whole, never sent truncated.
// ----------------------------------------------------------------------------
class Json {
 public:
  Json(char* buf, uint16_t cap) : _b(buf), _cap(cap) {}
  Json& begin(const char* type) { _n = 0; _ovf = false; _d = 0; _first[0] = true; put('{'); kvs("type", type); return *this; }
  Json& key(const char* k)                     { comma(); put('"'); raw(k); put('"'); put(':'); return *this; }
  Json& kv (const char* k, int32_t v)          { key(k); num(v); return *this; }
  Json& kvu(const char* k, uint32_t v)         { key(k); unum(v); return *this; }
  Json& kvf(const char* k, float v, uint8_t dp){ key(k); flt(v, dp); return *this; }
  Json& kvs(const char* k, const char* s)      { key(k); str(s); return *this; }
  Json& kvb(const char* k, bool b)             { key(k); put(b ? '1' : '0'); return *this; }  // flags travel as 0/1
  Json& obj(const char* k)                     { key(k); put('{'); push(); return *this; }
  Json& arr(const char* k)                     { key(k); put('['); push(); return *this; }
  Json& close(char c)                          { put(c); pop(); return *this; }               // '}' or ']'
  Json& ef(float v, uint8_t dp)                { comma(); flt(v, dp); return *this; }          // array items
  Json& eobj()                                 { comma(); put('{'); push(); return *this; }      // object inside an array
  Json& ei(int32_t v)                          { comma(); num(v); return *this; }
  Json& es(const char* s)                      { comma(); str(s); return *this; }
  uint16_t finish()                            { put('}'); put('\n'); return _ovf ? 0 : _n; }
  const char* data() const                     { return _b; }
 private:
  void comma()                 { if (_first[_d]) _first[_d] = false; else put(','); }
  void push()                  { if (_d < 7) { _d++; _first[_d] = true; } else _ovf = true; }
  void pop()                   { if (_d) _d--; }
  void put(char c)             { if (_n < _cap) _b[_n++] = c; else _ovf = true; }
  void raw(const char* s)      { while (*s) put(*s++); }
  void str(const char* s) {
    put('"');
    if (s) while (*s) { char c = *s++; if (c == '"' || c == '\\') { put('\\'); put(c); } else if ((uint8_t)c < 0x20) put(' '); else put(c); }
    put('"');
  }
  void u64(uint64_t v)         { char t[21]; uint8_t i = 0; do { t[i++] = (char)('0' + v % 10); v /= 10; } while (v); while (i) put(t[--i]); }
  void unum(uint32_t v)        { u64(v); }
  void num(int32_t v)          { if (v < 0) { put('-'); u64((uint64_t)(-(int64_t)v)); } else u64((uint64_t)v); }
  void flt(float v, uint8_t dp) {
    if (isnan(v) || isinf(v)) { raw("null"); return; }          // JSON has no NaN: null = "no value"
    if (dp > 6) dp = 6;
    uint64_t p = 1; for (uint8_t i = 0; i < dp; i++) p *= 10;
    bool neg = v < 0; double a = neg ? -(double)v : (double)v;
    if (a * (double)p > 1.0e18) { raw(neg ? "-1e18" : "1e18"); return; }
    uint64_t whole = (uint64_t)(a * (double)p + 0.5);
    if (neg && whole) put('-');
    u64(whole / p);
    if (dp) { put('.'); uint64_t f = whole % p; char t[6]; for (int8_t i = dp - 1; i >= 0; i--) { t[i] = (char)('0' + f % 10); f /= 10; } for (uint8_t i = 0; i < dp; i++) put(t[i]); }
  }
  char* _b; uint16_t _cap; uint16_t _n = 0; bool _ovf = false; uint8_t _d = 0; bool _first[8] = {true};
};

// ----------------------------------------------------------------------------
// TX queue. Serial.print() blocks when the UART buffer is full (a 550-char
// frame once froze the loop for ~47 ms every 100 ms). Every message goes into
// a ring here, and service() hands the UART only what it can take right now.
// Telemetry (isEvent=false) is dropped before events are.
// ----------------------------------------------------------------------------
namespace Tx {
  uint8_t  ring[TX_RING_BYTES];
  uint16_t head = 0, tail = 0;
  uint32_t drops = 0, dropsEvent = 0, sent = 0;
  inline uint16_t used() { return (uint16_t)((head + TX_RING_BYTES - tail) % TX_RING_BYTES); }
  inline uint16_t room() { return (uint16_t)(TX_RING_BYTES - 1 - used()); }
  bool push(const char* s, uint16_t n, bool isEvent) {
    uint16_t need = n + (isEvent ? 0 : TX_EVENT_RESERVE);
    if (n == 0 || need > room()) { if (isEvent) dropsEvent++; else drops++; return false; }
    for (uint16_t i = 0; i < n; i++) { ring[head] = (uint8_t)s[i]; head = (uint16_t)((head + 1) % TX_RING_BYTES); }
    return true;
  }
  void service() {
    int free = Serial.availableForWrite();
    while (free > 0 && head != tail) {
      uint16_t chunk = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(TX_RING_BYTES - tail);
      if (chunk > (uint16_t)free) chunk = (uint16_t)free;
      size_t w = Serial.write(&ring[tail], chunk);
      if (w == 0) break;
      tail = (uint16_t)((tail + w) % TX_RING_BYTES); free -= (int)w; sent += w;
    }
  }
}

// One shared frame buffer. Rule: build a message start-to-finish, then emit().
// Never call anything that emits while a message is half built.
char g_jbuf[JSON_BUF_BYTES];
Json J(g_jbuf, sizeof(g_jbuf));
bool emit(bool isEvent = true) { uint16_t n = J.finish(); if (!n) { Tx::dropsEvent++; return false; } return Tx::push(g_jbuf, n, isEvent); }
void logEvent(const char* lvl, const char* code, const char* msg = nullptr) {
  J.begin("event").kvu("ms", millis()).kvs("lvl", lvl).kvs("code", code);
  if (msg) J.kvs("msg", msg);
  emit();
}

// ----------------------------------------------------------------------------
// RX line reader: collects bytes until '\n' / '\r'. Never waits (the old
// readStringUntil froze the loop for a second). Over-long lines are dropped.
// ----------------------------------------------------------------------------
class LineReader {
 public:
  uint32_t overflows = 0;
  const char* poll() {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (_ovf) { _ovf = false; _n = 0; overflows++; continue; }
        if (_n == 0) continue;
        _b[_n] = 0; _n = 0; return _b;
      }
      if (_n < RX_LINE_MAX) _b[_n++] = c; else _ovf = true;
    }
    return nullptr;
  }
 private:
  char _b[RX_LINE_MAX + 1]; uint8_t _n = 0; bool _ovf = false;
};

// ----------------------------------------------------------------------------
// I2C bus manager.
//  STM32duino core 3.0.0 bug: after ANY NACK the Wire driver is stuck until
//  Wire.begin() runs again. So every failed transfer marks the bus dirty and
//  the next transfer re-initialises it first. A slave holding SDA low gets
//  9 clock pulses + a STOP (classic bus recovery).
// ----------------------------------------------------------------------------
namespace Bus {
  bool     dirty = false;
  uint32_t errors = 0, recoveries = 0;
  void begin() {
    Wire.setSDA(PIN_I2C_SDA); Wire.setSCL(PIN_I2C_SCL);   // BEFORE begin(): the default pins are PA8/PA9 = a lamp gate!
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    dirty = false;
  }
  inline void prep() { if (dirty) begin(); }
  inline void fail() { errors++; dirty = true; }
  bool sdaStuckLow() { return digitalRead(PIN_I2C_SDA) == LOW; }
  void recover() {                                      // free a slave that holds SDA low
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
    for (uint8_t i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
      digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
      digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);            // manual STOP: SDA low->high while SCL high
    digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);
    recoveries++;
    begin();
  }
  bool probe(uint8_t a) {                               // zero-length write: safe even on core 3.0.0
    prep(); Wire.beginTransmission(a);
    bool ok = Wire.endTransmission() == 0;
    if (!ok) dirty = true;
    return ok;
  }
  bool write(uint8_t a, const uint8_t* d, uint8_t n) {
    prep(); Wire.beginTransmission(a); Wire.write(d, n);
    if (Wire.endTransmission() != 0) { fail(); return false; }
    return true;
  }
  bool writeReg(uint8_t a, uint8_t reg, uint8_t v) { uint8_t b[2] = { reg, v }; return write(a, b, 2); }
  bool read(uint8_t a, uint8_t* d, uint8_t n) {
    prep();
    if (Wire.requestFrom((uint8_t)a, (uint8_t)n) != n) { fail(); return false; }
    for (uint8_t i = 0; i < n; i++) d[i] = (uint8_t)Wire.read();
    return true;
  }
  bool readReg(uint8_t a, uint8_t reg, uint8_t* d, uint8_t n) {      // pointer, REPEATED START, read
    prep(); Wire.beginTransmission(a); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { fail(); return false; }
    if (Wire.requestFrom((uint8_t)a, (uint8_t)n) != n) { fail(); return false; }
    for (uint8_t i = 0; i < n; i++) d[i] = (uint8_t)Wire.read();
    return true;
  }
  bool readRegStop(uint8_t a, uint8_t reg, uint8_t* d, uint8_t n) {  // pointer, STOP, read (what the AS7265x expects)
    uint8_t r = reg; if (!write(a, &r, 1)) return false;
    return read(a, d, n);
  }
}

// ----------------------------------------------------------------------------
// Store: small records in the emulated EEPROM (one flash page), each guarded
// by its own magic + version + size + CRC32. A corrupt or old record loads as
// "absent" and the caller falls back to defaults.
// STM32: the buffered API rewrites the page with ONE erase per save. Plain
// EEPROM.put() would erase the page once PER BYTE (seconds of blocking).
// The page is read into RAM once at boot (Store::begin), so saving one record
// keeps every other record in the page intact.
// Layout of a record: uint32 magic, uint16 version, uint16 size, ...data..., uint32 crc (last).
// ----------------------------------------------------------------------------
uint32_t crc32(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) { c ^= *p++; for (uint8_t k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  return ~c;
}
namespace Store {
  bool filled = false;
  void begin() {
#if defined(ARDUINO_ARCH_STM32)
    if (!filled) { eeprom_buffer_fill(); filled = true; }
#endif
  }
  struct Head { uint32_t magic; uint16_t version, size; };
  bool load(uint16_t off, void* rec, uint16_t size, uint32_t magic, uint16_t version) {
    uint8_t* p = (uint8_t*)rec;
    begin();
#if defined(ARDUINO_ARCH_STM32)
    for (uint16_t i = 0; i < size; i++) p[i] = eeprom_buffered_read_byte(off + i);
#else
    for (uint16_t i = 0; i < size; i++) p[i] = EEPROM.read(off + i);
#endif
    Head h; memcpy(&h, p, sizeof h);
    uint32_t crc; memcpy(&crc, p + size - 4, 4);
    return h.magic == magic && h.version == version && h.size == size && crc == crc32(p, size - 4);
  }
  void save(uint16_t off, void* rec, uint16_t size, uint32_t magic, uint16_t version) {   // ~50 ms of flash work: call while idle
    uint8_t* p = (uint8_t*)rec;
    Head h = { magic, version, size }; memcpy(p, &h, sizeof h);
    uint32_t crc = crc32(p, size - 4); memcpy(p + size - 4, &crc, 4);
    begin();
#if defined(ARDUINO_ARCH_STM32)
    for (uint16_t i = 0; i < size; i++) eeprom_buffered_write_byte(off + i, p[i]);
    eeprom_buffer_flush();
#else
    for (uint16_t i = 0; i < size; i++) EEPROM.update(off + i, p[i]);
#endif
  }
}

// ----------------------------------------------------------------------------
// Why did we boot? (read once, then cleared)
// ----------------------------------------------------------------------------
const char* readResetCause() {
#if defined(RCC_CSR_IWDGRSTF)
  uint32_t csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
  if (csr & RCC_CSR_IWDGRSTF) return "watchdog";
  if (csr & RCC_CSR_WWDGRSTF) return "window_watchdog";
  if (csr & RCC_CSR_SFTRSTF)  return "software";
  if (csr & RCC_CSR_BORRSTF)  return "power_on";
  if (csr & RCC_CSR_LPWRRSTF) return "low_power";
  if (csr & RCC_CSR_PINRSTF)  return "reset_button";
#endif
  return "unknown";
}
