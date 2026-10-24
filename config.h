#pragma once
// ============================================================================
//  TAUVER 2027 · SCIENCE PAYLOAD FIRMWARE 3.0 · config.h
//  ---------------------------------------------------------------------------
//  THE ONLY FILE A TEAMMATE SHOULD NEED TO EDIT.
//  Profile, which modules exist, pins, addresses, thresholds, timings.
//
//  Board : NUCLEO-G491RE  (Arduino IDE: STM32duino core 3.0.0,
//          "Generic STM32G4 series" -> "Generic G491RETx",
//          U(S)ART support: "Enabled (generic Serial)")
//  RULE  : pins are written with CHIP names (PA8, PB9 ...). On this variant a
//          bare number is a port-A pin (2 = PA2 = the USB serial line!) and
//          A2..A5 are PA2..PA5, not the header pins. Names only.
// ============================================================================
#include <Arduino.h>

#define FW_NAME           "tauver_science"
#define FW_VERSION        "3.0.0"
#define CONTRACT_VERSION  3          // bump when the JSON contract changes (GUI + bridge check it)

constexpr uint32_t NO_PIN = 0xFFFF;  // "this function has no pin wired"

// ----------------------------------------------------------------------------
// [A] PROFILE: which build is this?
//   BASE     = what scores points: 3 scales, lids, IMU, pH (+ liquid temp),
//              pumps, carbonate (SCD41 + acid + purge), MET (BME280 + wind)
//   STANDARD = BASE + reflectance (Triad AS7265x, VIS + NIR LED groups)
//   FULL     = STANDARD + microscope (ring light + Jetson capture handshake)
//   The SWIR hydration add-on is a stretch goal: off unless TAUVER_SWIR = 1.
// Pick here, or build with -DTAUVER_PROFILE=1|2|3 (tools/build.sh does that).
// Every module also has its own ENABLE_ line below: set one to false to cut
// it from ANY profile. A switched-off module is compiled in but never
// started; its mission steps skip silently and the GUI hides it.
// ----------------------------------------------------------------------------
#define PROFILE_BASE      1
#define PROFILE_STANDARD  2
#define PROFILE_FULL      3
#ifndef TAUVER_PROFILE
#define TAUVER_PROFILE    PROFILE_STANDARD
#endif
#ifndef TAUVER_SWIR
#define TAUVER_SWIR       0
#endif
constexpr uint8_t PROFILE = TAUVER_PROFILE;
static_assert(PROFILE >= PROFILE_BASE && PROFILE <= PROFILE_FULL, "TAUVER_PROFILE must be 1 (BASE), 2 (STANDARD) or 3 (FULL)");
#define PROFILE_NAME (PROFILE == PROFILE_BASE ? "BASE" : PROFILE == PROFILE_STANDARD ? "STANDARD" : "FULL")

// --- BASE: the point-earners ---
constexpr bool ENABLE_SCALES     = true;   // HX711 load cells, one per bay
constexpr bool ENABLE_DEEP_BAY   = true;   // the third bay: deep sample (REQ-FUN-130/160)
constexpr bool ENABLE_LIDS       = true;   // 28BYJ-48 + ULN2003 sliding lids, every bay (deep also needs ENABLE_DEEP_LID)
constexpr bool ENABLE_DEEP_LID   = true;   // optional lid on the deep bay
constexpr bool ENABLE_IMU        = true;   // MPU6050: steady gate + load-cell tilt fix
constexpr bool ENABLE_PH         = true;   // pH module (Astro-Bio)
constexpr bool ENABLE_PH_TEMP    = true;   // DS18B20 in the liquid: the pH slope needs the LIQUID temperature
constexpr bool ENABLE_PUMPS      = true;   // acid (general) + sample / rinse (astro-bio)
constexpr bool ENABLE_CO2        = true;   // SCD41 in the analysis chamber (carbonate test)
constexpr bool ENABLE_PURGE      = true;   // chamber purge fan / valve
constexpr bool ENABLE_MET        = true;   // BME280: temperature / humidity / pressure
constexpr bool ENABLE_WIND       = true;   // RS-485 Modbus ultrasonic anemometer
// --- ADD-ONS ---
constexpr bool ENABLE_REFLECT    = PROFILE >= PROFILE_STANDARD;
constexpr bool ENABLE_MICROSCOPE = PROFILE >= PROFILE_FULL;
constexpr bool ENABLE_SWIR       = TAUVER_SWIR;   // stretch: 1300 / 1450 nm hydration check
constexpr bool ENABLE_LIQUID_SENSE = false;       // optional: 2 electrodes at the intake tip ("am I in liquid?")
// --- SYSTEM ---
constexpr bool ENABLE_WATCHDOG   = true;   // IWDG: last-resort reboot if the loop hangs
constexpr bool ENABLE_ESTOP_IN   = true;   // local science-stop button input

// ----------------------------------------------------------------------------
// [B] HOST LINK  (ST-LINK virtual COM port = LPUART1 on PA2/PA3 -> USB -> Jetson / laptop)
// ----------------------------------------------------------------------------
constexpr uint32_t HOST_BAUD            = 115200;
constexpr uint16_t TELEMETRY_MS         = 100;     // fast frame, 10 Hz
constexpr uint8_t  SLOW_EVERY           = 5;       // every 5th fast frame carries the slow block (2 Hz)
constexpr uint16_t HEALTH_MS            = 2000;    // health frame, 0.5 Hz (~1 KB: the biggest regular frame)
constexpr uint32_t HOST_LINK_TIMEOUT_MS = 5000;    // no line from the host this long = link lost
constexpr uint16_t TX_RING_BYTES        = 8192;    // outgoing queue (never blocks the loop)
constexpr uint16_t TX_EVENT_RESERVE     = 1024;    // telemetry is dropped before events / results are
constexpr uint8_t  RX_LINE_MAX          = 96;      // longest accepted command line
constexpr uint16_t JSON_BUF_BYTES       = 2048;    // one frame (a reflectance result is the biggest, ~1.3 KB)

// ----------------------------------------------------------------------------
// [C] PIN MAP  (morpho = the long ST headers). 3.0 changes vs 2.0:
//   PA8 UVC gate -> VIS LED group · PA10 lid reed -> NIR LED group · PA0 MQ-4 -> SWIR photodiode
//   new: PA15 DS18B20 · PC10/PC11 HX711 deep · PA11/PA12/PC12/PD2 deep lid · PB4 deep end-stop
//        PB10 / PB0 SWIR LED gates · PC2 liquid sense (optional)
// A compile-time check at the bottom refuses to build if two ENABLED functions share a pin.
// ----------------------------------------------------------------------------
// --- inputs
constexpr uint32_t PIN_ESTOP        = PB3;   // D3  science-stop button -> GND (SWO pin: fine as input)
constexpr bool     ESTOP_ACTIVE_LOW = true;  // NO button to GND. For fail-safe NC wiring (open = stop) set false.
constexpr uint32_t PIN_LID_END_ROCK = NO_PIN;// optional lid "closed" end-stops (switch -> GND)
constexpr uint32_t PIN_LID_END_REG  = NO_PIN;
constexpr uint32_t PIN_LID_END_DEEP = PB4;   // D5  (wakes up pulled HIGH by JTAG: harmless for an input)
constexpr uint32_t PIN_LIQUID       = PC2;   // morpho: electrode pair to GND, internal pull-up (optional)
// --- reflectance lamp groups: logic-level N-MOSFET gate (+100 R series, 10 k gate -> GND)
constexpr uint8_t  N_ILLUM = 2;
constexpr uint32_t PIN_ILLUM[N_ILLUM]  = { PA8, PA10 };  // D7 = VIS group (white x2 + 405), D2 = NIR group (740 / 850 / 940)
const char* const  ILLUM_NAME[N_ILLUM] = { "VIS", "NIR" };
// --- pumps: H-bridge "IN/IN" inputs (DRV8871-style). IN1 = forward, IN2 = reverse/retract.
constexpr uint32_t PIN_ACID_IN1     = PC7;   // D9  TIM3_CH2  general module: HCl dosing
constexpr uint32_t PIN_ACID_IN2     = PB6;   // D10 TIM4_CH1  retract (anti-drip)
constexpr uint32_t PIN_SAMPLE_IN1   = PA9;   // D8  TIM1_CH2  astro-bio: liquid sample intake
constexpr uint32_t PIN_SAMPLE_IN2   = PB7;   // morpho TIM3_CH4  reverse: clear the intake tube
constexpr uint32_t PIN_RINSE_IN1    = PB11;  // morpho TIM2_CH4  astro-bio: DI-water rinse
constexpr uint32_t PIN_RINSE_IN2    = NO_PIN;//   rinse is forward-only
constexpr uint32_t PIN_PURGE        = PA4;   // A2 header (write PA4, never A2!) purge fan / valve gate
constexpr uint32_t PIN_MICRO_LIGHT  = PB2;   // morpho, microscope ring light gate
constexpr uint32_t PIN_AUTON_LIGHT  = NO_PIN;// optional autonomy indicator gate (REQ-FUN-070). The rover's
                                             // light belongs to Electrical; we publish the flag either way.
// --- HX711 load cells (power the HX711 from 3V3: these inputs are not 5 V tolerant)
constexpr uint32_t PIN_HX_ROCK_DOUT = PA6;   // D12
constexpr uint32_t PIN_HX_ROCK_SCK  = PA7;   // D11
constexpr uint32_t PIN_HX_REG_DOUT  = PC0;   // A5 header (write PC0)
constexpr uint32_t PIN_HX_REG_SCK   = PC1;   // A4 header (write PC1)
constexpr uint32_t PIN_HX_DEEP_DOUT = PC10;  // morpho
constexpr uint32_t PIN_HX_DEEP_SCK  = PC11;  // morpho
// --- lid steppers: ULN2003 IN1..IN4 (28BYJ-48)
constexpr uint32_t PIN_LID_ROCK[4]  = { PC6, PC8, PC9, PB12 };   // morpho
constexpr uint32_t PIN_LID_REG[4]   = { PB13, PB14, PB15, PB1 }; // morpho
constexpr uint32_t PIN_LID_DEEP[4]  = { PA11, PA12, PC12, PD2 }; // morpho (PA11/PA12 are free: USB is not wired)
// --- I2C1: Triad 0x49 · AS7341 0x39 · SCD4x 0x62 · MPU6050 0x68 · BME280 0x76
constexpr uint32_t PIN_I2C_SDA      = PB9;   // D14  (Wire defaults to PA8/PA9 = the VIS lamp gate! set first)
constexpr uint32_t PIN_I2C_SCL      = PB8;   // D15
constexpr uint32_t I2C_CLOCK_HZ     = 100000;// 100 kHz: long rover cables, conservative
// --- RS-485 (USART1) to the anemometer through a 3.3 V transceiver (MAX3485 class)
constexpr uint32_t PIN_485_RX       = PC5;   // D0
constexpr uint32_t PIN_485_TX       = PC4;   // D1
constexpr uint32_t PIN_485_DE       = PB5;   // D4  DE and /RE tied together
// --- 1-Wire: DS18B20 waterproof probe next to the pH electrode (4.7 k pull-up to 3V3)
constexpr uint32_t PIN_ONEWIRE      = PA15;  // morpho
// --- analog (0-3.3 V only! 5 V modules go through a 10k/20k divider)
constexpr uint32_t PIN_PH           = PA1;   // A1  pH module Po, via 10k/20k divider
constexpr uint32_t PIN_SWIR_PD      = PA0;   // A0  SWIR InGaAs photodiode amplifier (stretch)
constexpr uint32_t PIN_SWIR_1300    = PB10;  // D6  SWIR 1300 nm LED gate (stretch)
constexpr uint32_t PIN_SWIR_1450    = PB0;   // A3  SWIR 1450 nm LED gate (stretch)
constexpr float    ANALOG_DIVIDER   = 1.5f;  // (10k+20k)/20k : pin volts -> module volts
constexpr float    VDDA             = 3.3f;
// --- status LED
constexpr uint32_t PIN_HEARTBEAT    = PA5;   // D13 = green LD2

// ----------------------------------------------------------------------------
// [D] I2C ADDRESSES  (fixed by the chips; no clashes = no mux needed)
// ----------------------------------------------------------------------------
constexpr uint8_t ADDR_TRIAD  = 0x49;   // SparkFun Triad AS7265x
constexpr uint8_t ADDR_AS7341 = 0x39;   // fallback spectral sensor
constexpr uint8_t ADDR_SCD4X  = 0x62;
constexpr uint8_t ADDR_MPU    = 0x68;   // 0x69 if AD0 is tied high
constexpr uint8_t ADDR_BME280 = 0x76;   // 0x77 if SDO is tied high (Adafruit breakout)

// ----------------------------------------------------------------------------
// [E] HEALTH / RECOVERY
// ----------------------------------------------------------------------------
constexpr uint8_t  HEALTH_FAIL_LIMIT   = 3;      // consecutive errors before OFFLINE
constexpr uint32_t HEALTH_STARTUP_MS   = 2500;   // grace for first good data after (re)start
constexpr uint32_t HEALTH_RETRY_MIN_MS = 1000;   // OFFLINE retry back-off: 1 s -> 2 -> 4 -> 8
constexpr uint32_t HEALTH_RETRY_MAX_MS = 8000;
constexpr uint32_t WATCHDOG_US         = 2000000;// 2 s

// ----------------------------------------------------------------------------
// [F] SAMPLE BAYS & SCALES
// ----------------------------------------------------------------------------
constexpr uint8_t  N_BAYS             = 3;       // ROCK, REGOLITH, DEEP
constexpr float    SAMPLE_TARGET_G    = 100.0f;  // ERC REQ-FUN-090: >= 100 g per sample
constexpr float    SAMPLE_MIN_G       = 5.0f;    // below this the bay counts as empty
constexpr uint8_t  SCALE_WINDOW       = 10;      // HX711 @10 SPS -> 1 s moving window
constexpr float    SCALE_STABLE_G     = 1.0f;    // window spread for "stable" (REQ: 1 g accuracy)
constexpr uint32_t SCALE_TIMEOUT_MS   = 1000;    // no conversion this long = fault
constexpr float    SCALE_TILT_MAX_DEG = 15.0f;   // beyond this, tilt compensation is refused
constexpr uint8_t  ANALYSIS_BAY       = 1;       // REGOLITH: the reflectance probe + CO2 sensor live in its lid

// ----------------------------------------------------------------------------
// [G] LIDS (28BYJ-48 through ULN2003, full-step two-phase = max torque)
// ----------------------------------------------------------------------------
constexpr int32_t  LID_TRAVEL_STEPS[N_BAYS] = { 4096, 4096, 4096 }; // closed->open. 2048 steps = 1 output rev
constexpr bool     LID_INVERT[N_BAYS]       = { false, false, false };
constexpr uint16_t LID_STEP_US              = 2500;   // 400 steps/s ~ 11.7 rpm (28BYJ max ~15)
constexpr bool     LID_CLOSE_NEEDS_ARM_CLEAR = true;  // don't close a lid on the gripper
constexpr int32_t  LID_JOG_STEPS            = 200;    // CMD:SELFTEST=MOTION: small open-and-back jog

// ----------------------------------------------------------------------------
// [H] PUMPS  (calibrate each with CMD:PUMP_CAL; until then volume doses are refused)
// ----------------------------------------------------------------------------
constexpr uint8_t  PUMP_DEFAULT_DUTY  = 200;     // 0..255
constexpr uint16_t PUMP_RAMP_MS       = 150;     // soft start: less inrush, less splashing
constexpr uint32_t PUMP_MAX_RUN_MS    = 60000;   // hard ceiling for any single run
constexpr uint16_t PUMP_RETRACT_MS    = 400;     // reverse after a dose so the line can't drip
constexpr float    ASTRO_MAX_SAMPLE_ML= 90.0f;   // ERC Astro-Bio: sample must be < 100 mL
constexpr uint32_t PWM_FREQ_HZ        = 1000;

// ----------------------------------------------------------------------------
// [I] REFLECTANCE  (reflect.h, shared byte-for-byte with the Reflectance Bench)
// ----------------------------------------------------------------------------
constexpr uint8_t  REFLECT_CHIP_PREF     = 0;     // 0 = auto (Triad first, then AS7341) · 1 = Triad only · 2 = AS7341 only
constexpr uint32_t REFLECT_TRIAD_BOOT_MS = 1500;  // the Triad needs ~1 s after power-up before it answers
// Chip LEDs lit WITH each group: 0 on the rover. The probe head's external LEDs do the lighting; the
// Triad's own LEDs sit beside the sensor and would light at 0° = the glint geometry the head removes.
constexpr uint8_t  ILLUM_CHIPLED[N_ILLUM] = { 0, 0 };
constexpr uint8_t  TRIAD_LED_CUR  = 1;            // Triad chip LED drive: 0 = 12.5, 1 = 25, 2 = 50, 3 = 100 mA
constexpr uint8_t  AS7341_LED_MA  = 20;           // AS7341 board LED drive, 4..258 mA
constexpr uint8_t  REFLECT_FRAMES = 3;            // lit frames per group
constexpr uint8_t  REFLECT_FRAMES_MAX = 8;
constexpr uint8_t  REFLECT_GROUPS = 0b11;         // groups used by WHITE / SCAN
constexpr uint16_t REFLECT_SETTLE_MS = 5;         // lamp on -> wait -> integrate
constexpr float    REFLECT_AE_LO = 0.25f, REFLECT_AE_HI = 0.85f, REFLECT_AE_TARGET = 0.55f;
constexpr uint8_t  REFLECT_AE_TRIES = 8;
constexpr float    REFLECT_TINT_START_MS = 50.0f;
constexpr float    REFLECT_TINT_MAX_MS   = 300.0f;
constexpr uint32_t REFLECT_LAMP_CAP_MS   = 20000; // no single burn longer than this
constexpr uint32_t LIVE_LIT_MAX_MS  = 20000;
constexpr uint32_t LIVE_DARK_MAX_MS = 60000;
constexpr float    REFLECT_W_MIN_FRAC = 0.02f;    // a band its group lights at < 2 % of the brightest = unlit
constexpr float    REFLECT_W_MIN_SNR  = 10.0f;    // ... or whose white is < 10x its own noise
constexpr uint16_t REFLECT_WHITE_STORE_OFF = 1024;// the white's slot in the emulated EEPROM page (cal lives at 0)
constexpr float    WHITE_MAX_CUP_G    = SAMPLE_MIN_G; // WHITE needs an EMPTY cup: the PTFE floor is the white
// SELFTEST lamp loop: each group must raise its own bands by at least this many counts over dark
constexpr float    LAMPTEST_MIN_COUNTS = 200.0f;

// ----------------------------------------------------------------------------
// [J] CARBONATE  (HCl + SCD41: detection only, by physics, see the Logic Sweep)
// ----------------------------------------------------------------------------
constexpr uint32_t CARB_BASELINE_MS     = 30000; // flat CO2 for 30 s or the chamber leaks
constexpr int32_t  CARB_DRIFT_MAX_PPM   = 40;    // max-min allowed during baseline
constexpr int32_t  CARB_BASELINE_MAX_PPM= 2000;  // chamber not purged -> refuse to drip
constexpr float    CARB_DOSE_ML         = 0.5f;  // acid dose (needs pump cal)
constexpr uint16_t CARB_DOSE_MS_UNCAL   = 500;   // used only if the acid pump is uncalibrated
constexpr uint32_t CARB_REACT_MS        = 150000;// watch 2.5 min (SCD4x tau63 ~ 60 s)
constexpr uint32_t CARB_PURGE_MS        = 60000;
constexpr int32_t  CARB_DETECT_PPM      = 100;   // delta above baseline = "carbonate present"
constexpr int32_t  CO2_OVER_RANGE_PPM   = 5000;  // SCD41 accuracy spec ends here

// ----------------------------------------------------------------------------
// [K] pH  (Nernst slope is temperature dependent: 54.2 / 59.2 / 61.2 mV/pH at 0/25/35 C)
// ----------------------------------------------------------------------------
constexpr float    PH_DEFAULT_V7      = 2.50f;   // module volts at pH 7 (typical PH-4502C)
constexpr float    PH_DEFAULT_SLOPE   = -0.177f; // module volts per pH unit (inverting front end)
constexpr uint8_t  PH_WINDOW          = 20;      // 10 Hz -> 2 s median window
constexpr float    PH_STABLE_DRIFT    = 0.03f;   // median may wander this much (pH) ...
constexpr uint32_t PH_STABLE_HOLD_MS  = 5000;    // ... for this long before a reading counts as settled
constexpr float    PH_FALLBACK_TEMP_C = 25.0f;   // used if neither the liquid nor the air temperature is known
constexpr uint32_t ONEWIRE_PERIOD_MS  = 2000;    // DS18B20: one 12-bit conversion (750 ms) every 2 s
constexpr uint32_t LIQUID_HOLD_MS     = 300;     // liquid sense: electrodes bridged this long = in liquid

// ----------------------------------------------------------------------------
// [L] IMU & STEADY GATE
// ----------------------------------------------------------------------------
constexpr float    STEADY_TILT_DEG    = 15.0f;
constexpr float    STEADY_RATE_DPS    = 3.0f;
constexpr float    STEADY_G_TOL       = 0.06f;   // |a| within 1 g +- this
constexpr uint32_t STEADY_HOLD_MS     = 500;     // must hold steady this long

// ----------------------------------------------------------------------------
// [M] WIND (Modbus RTU). Register map differs per model: CHECK YOUR DATASHEET.
// ----------------------------------------------------------------------------
constexpr uint32_t WIND_BAUD          = 9600;
constexpr uint8_t  WIND_ADDR          = 0x01;
constexpr uint16_t WIND_REG_START     = 0x0000;
constexpr uint8_t  WIND_REG_COUNT     = 2;
constexpr uint8_t  WIND_SPEED_IDX     = 0;       // register index (from START) holding speed
constexpr float    WIND_SPEED_SCALE   = 0.01f;   // m/s per LSB
constexpr uint8_t  WIND_DIR_IDX       = 1;       // register index holding direction
constexpr float    WIND_DIR_SCALE     = 1.0f;    // degrees per LSB
constexpr uint32_t WIND_POLL_MS       = 1000;
constexpr uint32_t WIND_REPLY_TIMEOUT_MS = 150;

// ----------------------------------------------------------------------------
// [N] SWIR hydration add-on (stretch)
// ----------------------------------------------------------------------------
constexpr uint8_t  SWIR_FRAMES        = 4;       // lit frames per wavelength (each bracketed by darks)
constexpr uint16_t SWIR_SETTLE_MS     = 5;
constexpr uint8_t  SWIR_OVERSAMPLE    = 32;      // ADC reads averaged per frame
constexpr float    SWIR_MIN_SIGNAL_V  = 0.02f;   // white signal below this = lamp or amplifier dead

// ----------------------------------------------------------------------------
// [O] MODES, MISSIONS, EXTERNAL STATE
// ----------------------------------------------------------------------------
constexpr uint32_t ARM_DELAY_MS        = 5000;   // REQ-OPS-090: autonomy starts >= 5 s after the start command
constexpr uint32_t CONFIRM_TIMEOUT_MS  = 60000;  // unanswered confirm = DENY (fail safe, never open)
constexpr uint32_t CONFIRM_REPEAT_MS   = 5000;   // re-announce a pending confirm
constexpr uint32_t EXT_FRESH_MS        = 3000;   // ROS2 mode: STATE:* older than this counts as unknown
constexpr uint32_t EXT_STATE_STALE_MS  = 10000;  // reported stale in the health frame
constexpr int16_t  BATT_MIN_SECONDARY_PCT = 30;  // SECONDARY steps are skipped below this

// ============================================================================
// PIN-CONFLICT CHECK: refuses to compile if two ENABLED functions share a pin.
// (With 20+ functions on one Nucleo this is the classic silent bug.)
// ============================================================================
struct PinUse { uint32_t pin; bool used; };
constexpr PinUse PIN_TABLE[] = {
  { PA2, true }, { PA3, true },                                   // host serial (ST-LINK VCP)
  { PA13, true }, { PA14, true },                                 // SWD: never touch
  { PIN_HEARTBEAT, true },
  { PIN_I2C_SDA, true }, { PIN_I2C_SCL, true },
  { PIN_ESTOP, ENABLE_ESTOP_IN },
  { PIN_ILLUM[0], ENABLE_REFLECT }, { PIN_ILLUM[1], ENABLE_REFLECT },
  { PIN_ACID_IN1, ENABLE_PUMPS }, { PIN_ACID_IN2, ENABLE_PUMPS },
  { PIN_SAMPLE_IN1, ENABLE_PUMPS }, { PIN_SAMPLE_IN2, ENABLE_PUMPS },
  { PIN_RINSE_IN1, ENABLE_PUMPS }, { PIN_RINSE_IN2, ENABLE_PUMPS },
  { PIN_PURGE, ENABLE_PURGE }, { PIN_MICRO_LIGHT, ENABLE_MICROSCOPE }, { PIN_AUTON_LIGHT, true },
  { PIN_HX_ROCK_DOUT, ENABLE_SCALES }, { PIN_HX_ROCK_SCK, ENABLE_SCALES },
  { PIN_HX_REG_DOUT, ENABLE_SCALES }, { PIN_HX_REG_SCK, ENABLE_SCALES },
  { PIN_HX_DEEP_DOUT, ENABLE_SCALES && ENABLE_DEEP_BAY }, { PIN_HX_DEEP_SCK, ENABLE_SCALES && ENABLE_DEEP_BAY },
  { PIN_LID_ROCK[0], ENABLE_LIDS }, { PIN_LID_ROCK[1], ENABLE_LIDS }, { PIN_LID_ROCK[2], ENABLE_LIDS }, { PIN_LID_ROCK[3], ENABLE_LIDS },
  { PIN_LID_REG[0], ENABLE_LIDS }, { PIN_LID_REG[1], ENABLE_LIDS }, { PIN_LID_REG[2], ENABLE_LIDS }, { PIN_LID_REG[3], ENABLE_LIDS },
  { PIN_LID_DEEP[0], ENABLE_LIDS && ENABLE_DEEP_LID }, { PIN_LID_DEEP[1], ENABLE_LIDS && ENABLE_DEEP_LID },
  { PIN_LID_DEEP[2], ENABLE_LIDS && ENABLE_DEEP_LID }, { PIN_LID_DEEP[3], ENABLE_LIDS && ENABLE_DEEP_LID },
  { PIN_LID_END_ROCK, ENABLE_LIDS }, { PIN_LID_END_REG, ENABLE_LIDS }, { PIN_LID_END_DEEP, ENABLE_LIDS && ENABLE_DEEP_LID },
  { PIN_485_RX, ENABLE_WIND }, { PIN_485_TX, ENABLE_WIND }, { PIN_485_DE, ENABLE_WIND },
  { PIN_ONEWIRE, ENABLE_PH_TEMP }, { PIN_PH, ENABLE_PH }, { PIN_LIQUID, ENABLE_LIQUID_SENSE },
  { PIN_SWIR_PD, ENABLE_SWIR }, { PIN_SWIR_1300, ENABLE_SWIR }, { PIN_SWIR_1450, ENABLE_SWIR },
};
constexpr bool pinsUnique() {
  constexpr size_t n = sizeof(PIN_TABLE) / sizeof(PIN_TABLE[0]);
  for (size_t i = 0; i < n; i++)
    for (size_t j = i + 1; j < n; j++)
      if (PIN_TABLE[i].used && PIN_TABLE[j].used && PIN_TABLE[i].pin != NO_PIN && PIN_TABLE[i].pin == PIN_TABLE[j].pin) return false;
  return true;
}
static_assert(pinsUnique(), "PIN CONFLICT: two enabled functions share a pin. Check section [C] of config.h");
