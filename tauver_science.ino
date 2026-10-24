/*
 * ============================================================================
 *  TAUVER 2027 | SCIENCE PAYLOAD FIRMWARE  v3.0.0          (tauver_science.ino)
 *  Board  : NUCLEO-G491RE (STM32G491RE, Cortex-M4F @ 170 MHz)
 *  IDE    : Arduino IDE + STM32duino core 3.0.0
 *           Tools > Board > STM32 MCU based boards > Generic STM32G4 series
 *           Tools > Board part number > Generic G491RETx
 *           Tools > U(S)ART support > Enabled (generic Serial)
 *           Upload: STM32CubeProgrammer (SWD), or drag the .bin onto NOD_G491RE
 *  Libraries: NONE beyond the core (Wire, EEPROM, IWatchdog ship with it).
 *  Lead   : Plan Master Lev  ·  Co-pilot: NIX
 * ----------------------------------------------------------------------------
 *  "Objects inside the box, messages between the boxes, a table to drive the dance."
 *
 *    config.h      profile, modules, pins, thresholds: the ONLY file to edit
 *    core.h        JSON writer, TX queue, line reader, I2C bus, flash store   (shared with the bench)
 *    subsystem.h   the sensor contract + health state machine                (shared with the bench)
 *    state.h       modes, policy, lock, external state, stop button, calibration
 *    actuators.h   pumps, timed outputs, stepper lids
 *    sensors.h     IMU, 3x HX711, BME280, SCD4x, RS-485 wind, DS18B20, pH, liquid sense
 *    reflect.h     the reflectance instrument: Triad / AS7341, lamps, white, scan (shared with the bench)
 *    swir.h        SWIR hydration add-on (stretch)
 *    procedures.h  bays, the analysis bay, dosing, the carbonate test, self-test
 *    mission.h     the mission TABLES, the sequencer, SAFE
 *    commands.h    the command table, telemetry / health / boot frames
 *
 *  The firmware only reads and writes JSON lines over USB serial. It knows
 *  nothing about ROS: the Jetson bridge turns these lines into ROS2 topics.
 *  The loop never blocks: every module does a small slice of work and returns.
 * ============================================================================
 */
#include "config.h"
#include "core.h"
#include "state.h"
#include "subsystem.h"
#include "actuators.h"
#include "sensors.h"
#include "procedures.h"
#include "mission.h"
#include "commands.h"
#include <IWatchdog.h>

uint32_t nextTele = 0, nextHealth = 0, lastBeat = 0;
bool     beat = false, hostWasUp = false;

void setup() {
  // 1) Every actuator pin LOW before anything else can happen. (MOSFET gates also
  //    carry 10k pull-downs: a pin is undefined until this line runs.)
  for (uint8_t g = 0; g < N_ILLUM; g++) pinOutLow(PIN_ILLUM[g]);
  if (ENABLE_SWIR) { pinOutLow(PIN_SWIR_1300); pinOutLow(PIN_SWIR_1450); }
  pinOutLow(PIN_PURGE); pinOutLow(PIN_MICRO_LIGHT); pinOutLow(PIN_AUTON_LIGHT); pinOutLow(PIN_485_DE);
  for (Pump* p : PUMPS) p->init();
  for (StepperLid* l : LIDS) l->init();
  purge.init(); microLight.init(); autonLight.init(); illum.begin();
  pinOutLow(PIN_HEARTBEAT);

  const char* reset = readResetCause();
  Serial.begin(HOST_BAUD);
  analogReadResolution(12);                          // STM32duino defaults to 10 bits
  analogWriteFrequency(PWM_FREQ_HZ);
  Safety::init();
  Bus::begin();
  Store::begin();
  bool calOk = calLoad();

  for (uint8_t k = 0; k < N_MISSIONS; k++) {         // the mission tables are checked, not trusted
    const char* why = "";
    missionValid[k] = validateMission(MISSIONS[k], why);
    if (!missionValid[k]) { char msg[64]; snprintf(msg, sizeof msg, "%s: %s", MISSIONS[k].name, why); logEvent("error", "MISSION_TABLE_INVALID", msg); }
  }
  sendBoot(reset, calOk);
  if (!calOk) logEvent("warn", "CAL_DEFAULTED", "no valid calibration in flash: scales/pH/pumps uncalibrated");
  if (!strcmp(reset, "watchdog")) logEvent("error", "WATCHDOG_RESET", "the loop hung > 2 s last run");
  for (uint8_t k = 0; k < N_MISSIONS; k++) emitMissionDef(MISSIONS[k], missionValid[k]);

  nextTele = millis(); nextHealth = millis() + 500;
  if (ENABLE_WATCHDOG) IWatchdog.begin(WATCHDOG_US);
}

void loop() {
  uint32_t t0 = micros();
  if (ENABLE_WATCHDOG) IWatchdog.reload();
  Tx::service();

  // ---- inputs: commands, the stop button ----
  if (const char* line = rx.poll()) { g_lastHostRx = millis(); handleLine(line); }
  Safety::poll();
  if (Safety::estop && g_state != ST_SAFE) enterSafe("ESTOP_BUTTON");

  // ---- host link watchdog: a dropped confirm must fail SAFE, never open ----
  g_hostLinkUp = g_lastHostRx != 0 && since(g_lastHostRx) < HOST_LINK_TIMEOUT_MS;
  if (hostWasUp && !g_hostLinkUp) {
    logEvent("warn", "HOST_LINK_LOST", "no line from the host for 5 s");
    const char* w;
    if (g_state == ST_CONFIRM) seq.confirm(false, w);
    if (g_mode == MODE_ROS2 && seq.running()) seq.abort("HOST_LINK_LOST");   // ROS2 runs need the bridge's state
  }
  hostWasUp = g_hostLinkUp;

  // ---- sensors (each one: init/retry, or a slice of polling) ----
  for (Subsystem* s : SUBS) s->service();

  // ---- actuators and procedures ----
  for (Pump* p : PUMPS) p->poll();
  for (StepperLid* l : LIDS) l->poll();
  purge.poll(); microLight.poll();
  dose.poll(); carb.poll(); selftest.poll();
  seq.service();

  // ---- autonomy indicator (REQ-FUN-070): the flag is in telemetry; the pin is optional ----
  bool auton = seq.autonomous();
  if (auton != autonLight.isOn()) { const char* w; if (auton) autonLight.on(0, w); else autonLight.off(); }

  // ---- frames out (fixed rate: next += period, never "+= late") ----
  if (reached(nextTele))   { nextTele += TELEMETRY_MS; if (reached(nextTele)) nextTele = millis() + TELEMETRY_MS; sendTelemetry(); }
  if (reached(nextHealth)) {
    nextHealth += HEALTH_MS; if (reached(nextHealth)) nextHealth = millis() + HEALTH_MS;
    co2.setAmbientPressure(met.hPa());               // BME280 pressure -> SCD4x compensation
    sendHealth();
  }

  // ---- heartbeat: 1 Hz idle, 4 Hz armed / mission, 10 Hz SAFE ----
  uint32_t half = g_state == ST_SAFE ? 50 : (g_state == ST_IDLE ? 500 : 125);
  if (since(lastBeat) >= half) { lastBeat = millis(); beat = !beat; digitalWrite(PIN_HEARTBEAT, beat ? HIGH : LOW); }

  Tx::service();
  uint32_t dt = sinceUs(t0); if (dt > g_loopMaxUs) g_loopMaxUs = dt;
}
