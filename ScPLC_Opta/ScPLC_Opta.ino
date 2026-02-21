// ScPLC Opta main sketch
// Non-blocking state machine for measurement, aeration, and scrubbing cycles.

#include <Arduino.h>
#include "io_pins.h"

// Network / Modbus
#include <Ethernet.h>
// ArduinoModbus depends on ArduinoRS485 (even for TCP)
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Nonvolatile calibration storage (internal flash)
// Opta uses Arduino Mbed OS core; FlashIAP lives under mbed:: and is declared in drivers/FlashIAP.h.
#if defined(ARDUINO_ARCH_MBED)
  #include <drivers/FlashIAP.h>
  using mbed::FlashIAP;
#else
  #error "Calibration persistence requires an Mbed-based core (FlashIAP)."
#endif

#include <stdlib.h>
 
// Note: EthernetServer is provided by <Ethernet.h>; no extra include needed

// Opta expansions (A0602 analog, D1608E digital)
// Force the same library used by your working example sketches
#include <OptaBlue.h>
using namespace Opta;
#define HAVE_OPTA_BLUE 1

// -----------------------------------------------------------------------------
// Logging controls (trim noisy output when stable)
// -----------------------------------------------------------------------------
static const bool LOG_MB_FRAMES   = false; // [MB-REQ]/[MB-RSP]/[MB-0x03] frame logs
static const bool LOG_ACTIONS     = false; // "Action: ..." state/action messages
static const bool LOG_RELAY_ECHO  = false; // low-level relay echo prints
static const bool LOG_HEARTBEAT   = true;  // periodic heartbeat / readings
static const bool LOG_DISCOVERY   = true;  // expansion discovery/rescan logs
static const bool LOG_SWEEP_EVENTS= true;  // sweep BEGIN/STOP messages
static const bool LOG_A0602_DIAG  = true;  // once-per-second analog mode/raw/mA/% logs

// Use ArduinoModbus for a standards-compliant Modbus TCP server.
// Set to 0 to temporarily fall back to the legacy custom parser.
#define USE_ARDUINO_MODBUS 1

// Forward declarations to placate Arduino's auto-prototype generation
enum class Step : uint8_t; // full definition follows below

// Global Modbus TCP server instance
EthernetServer    dbgServer(5050);
EthernetServer    reachServer(5503); // temporary reachability test server
EthernetServer    mbServer(502); // Modbus TCP server socket
ModbusTCPServer   modbusTCPServer;

// Holding register mirror (0-based addresses). Sized to cover the largest block we expose.
// Keep this modest; 200 regs = 400 bytes.
static const uint16_t HOLDING_REGS_SIZE = 200;
static uint16_t holdingRegs[HOLDING_REGS_SIZE] = {0};

// Modbus health instrumentation (ultra-light)
// - mbPollCount increments each time modbusTCPServer.poll() is called
// - mbReqCount increments when poll() reports a handled request
// Printed at 1 Hz so we can confirm servicing continues even if a client can't read.
static uint32_t mbPollCount = 0;
static uint32_t mbReqCount  = 0;
static uint32_t mbLastReqMs = 0;

#if USE_ARDUINO_MODBUS
// Modbus TCP single-client idle timeout.
// If a client holds the TCP socket open but stops sending Modbus requests (e.g., HMI sleep),
// we drop it so another client (e.g., Pi) can connect.
static const uint32_t MB_IDLE_TIMEOUT_MS = 5000u;

// Deferred RO restore flag.
// We intentionally do NOT write holding registers back in the same loop() iteration as
// modbusTCPServer.poll(), because doing so can corrupt the in-flight Modbus TCP response
// (observed as connection resets / "RST by peer").
static bool mbRoRestorePending = false;
// When a RO mismatch is detected, we "arm" a restore for a later loop iteration.
// The restore is allowed only after at least one *additional* poll() has occurred.
static uint32_t mbRoRestoreArmedAtPollCount = 0;
#endif

static inline void mb_write(uint16_t addr, uint16_t value) {
  if (addr < HOLDING_REGS_SIZE) holdingRegs[addr] = value;
#if USE_ARDUINO_MODBUS
  // Mirror into ArduinoModbus mapping.
  (void)modbusTCPServer.holdingRegisterWrite((int)addr, value);
#endif
}

static inline bool isHoldingRegWritable(uint16_t addr) {
  // RW config/setpoints
  if (addr >= 30 && addr <= 35) return true; // O2 calibration/config
  if (addr >= 40 && addr <= 45) return true; // CO2 calibration/config
  if (addr == 52) return true; // sweep relay ON dwell time (ms)

  // Control ownership arbitration (reserved for Option B later)
  if (addr == 100 || addr == 101 || addr == 105) return true; // CONTROL_OWNER / CONTROL_CMD_ID / HMI_HEARTBEAT

  // CMD register (write triggers action; firmware clears)
  if (addr == 50) return true;

  return false; // default RO
}

#if USE_ARDUINO_MODBUS
static inline bool mb_ro_violation_in_range(uint16_t startAddr, uint16_t endAddr) {
  if (endAddr < startAddr) return false;
  if (startAddr >= HOLDING_REGS_SIZE) return false;
  if (endAddr >= HOLDING_REGS_SIZE) endAddr = (uint16_t)(HOLDING_REGS_SIZE - 1);

  for (uint16_t addr = startAddr; addr <= endAddr; addr++) {
    if (isHoldingRegWritable(addr)) continue;
    long current = modbusTCPServer.holdingRegisterRead((int)addr);
    if (current < 0) continue;
    if ((uint16_t)current != holdingRegs[addr]) return true;
  }
  return false;
}

static inline bool mb_detect_ro_violations() {
  // Keep this lightweight: only scan known RO ranges.
  if (mb_ro_violation_in_range(5, 9)) return true;
  if (mb_ro_violation_in_range(10, 11)) return true;
  if (mb_ro_violation_in_range(19, 20)) return true;
  if (mb_ro_violation_in_range(60, 67)) return true;
  if (mb_ro_violation_in_range(70, 71)) return true;
  if (mb_ro_violation_in_range(180, 181)) return true;
  if (mb_ro_violation_in_range(189, 198)) return true;
  if (mb_ro_violation_in_range(51, 51)) return true;
  return false;
}

static inline void mb_restore_ro_range(uint16_t startAddr, uint16_t endAddr) {
  if (endAddr < startAddr) return;
  if (startAddr >= HOLDING_REGS_SIZE) return;
  if (endAddr >= HOLDING_REGS_SIZE) endAddr = (uint16_t)(HOLDING_REGS_SIZE - 1);

  for (uint16_t addr = startAddr; addr <= endAddr; addr++) {
    if (isHoldingRegWritable(addr)) continue;
    (void)modbusTCPServer.holdingRegisterWrite((int)addr, holdingRegs[addr]);
  }
}

static inline void mb_restore_ro_holding_registers() {
  // Restore only known RO ranges.
  mb_restore_ro_range(5, 9);
  mb_restore_ro_range(10, 11);
  mb_restore_ro_range(19, 20);
  mb_restore_ro_range(60, 67);
  mb_restore_ro_range(70, 71);
  mb_restore_ro_range(180, 181);
  mb_restore_ro_range(189, 198);
  mb_restore_ro_range(51, 51);
}
#endif

// EthernetClient helpers:
// - This Opta EthernetClient type doesn't support operator==.
// - Some methods (connected/remoteIP) are non-const.
// We only use remoteIP-based matching to avoid re-adding the same connected client
// to multiple slots when EthernetServer.available() returns an already-connected client.
static inline bool mb_samePeer(EthernetClient& a, EthernetClient& b) {
  if (!a.connected()) return false;
  if (!b.connected()) return false;
  return a.remoteIP() == b.remoteIP();
}

// Periodic expansion rescan to handle late power-up of backplane
static unsigned long lastExpansionRescanMs = 0;
// Forward references to expansion indices defined later in this file
extern int8_t analogExpIndex;
extern int8_t digitalExpIndex;
static void tryRescanExpansions(){
  #ifdef HAVE_OPTA_BLUE
  unsigned long now = millis();
  if (now - lastExpansionRescanMs < 2000) return; // every 2s
  lastExpansionRescanMs = now;
  bool needAnalog = (analogExpIndex < 0);
  bool needDigital = (digitalExpIndex < 0);
  if (!needAnalog && !needDigital) return;
  OptaController.update();
  for (int i = 0; i < OPTA_CONTROLLER_MAX_EXPANSION_NUM; i++) {
    if (needAnalog) {
      AnalogExpansion ae = OptaController.getExpansion(i);
      if (ae && ae.getType() == EXPANSION_OPTA_ANALOG) {
        analogExpIndex = i;
        AnalogExpansion::beginChannelAsCurrentAdc(OptaController, analogExpIndex, 0);
        AnalogExpansion::beginChannelAsCurrentAdc(OptaController, analogExpIndex, 1);
        Serial.print("[Rescan] Found A0602 at index "); Serial.print(i);
        Serial.print(" I2C="); Serial.println(ae.getI2CAddress());
        needAnalog = false;
      }
    }
    if (needDigital) {
      DigitalMechExpansion mech = OptaController.getExpansion(i);
      DigitalStSolidExpansion sts = OptaController.getExpansion(i);
      if (mech) {
        digitalExpIndex = i;
        Serial.print("[Rescan] Found Digital expansion at index "); Serial.print(i);
        Serial.print(" type=MEC I2C="); Serial.println(mech.getI2CAddress());
        needDigital = false;
      } else if (sts) {
        digitalExpIndex = i;
        Serial.print("[Rescan] Found Digital expansion at index "); Serial.print(i);
        Serial.print(" type=STS I2C="); Serial.println(sts.getI2CAddress());
        needDigital = false;
      }
    }
  }
  #endif
}

// -----------------------------------------------------------------------------
// Process state enums
// -----------------------------------------------------------------------------

enum class RoomMode {
  Idle,
  Measuring,
  Aerating,
  ScrubbingCfg1,
  ScrubbingCfg2,
  Calibrating
};

enum class Step : uint8_t {
  None,
  Meas,
  Calibrate,
  AerationPart1,
  AerationPart2,
  Cfg1_AdsReg1,
  Cfg1_AdsReg2,
  Cfg1_XtrRegen,
  Cfg1_Wait,
  Cfg1_RmMtFill,
  Cfg2_AdsReg1,
  Cfg2_AdsReg2,
  Cfg2_XtrRegen,
  Cfg2_Wait,
  Cfg2_RmMtFill
};

// -----------------------------------------------------------------------------
// Timing constants (ms)
// -----------------------------------------------------------------------------

// Measurement
const unsigned long MEASUREMENT_TIME_MS = 120000UL; // 120 s
// Calibration
const unsigned long CALIBRATION_TIME_MS = 600000UL; // 10 minutes

// Aeration
const unsigned long AER_PART1_TIME_MS = 10000UL;    // 10 s
const unsigned long AER_PER_0_1PCT_MS = 20000UL;    // 20 s per 0.1% below setpoint

// Scrub Configuration 1
const unsigned long CFG1_ADSREG1_MS  = 50000UL;
const unsigned long CFG1_ADSREG2_MS  = 310000UL;
const unsigned long CFG1_XTRREGEN_MS = 230000UL;
const unsigned long CFG1_WAIT_MS     = 10000UL;
const unsigned long CFG1_RMMTFILL_MS = 25000UL;

// Scrub Configuration 2
const unsigned long CFG2_ADSREG1_MS  = 50000UL;
const unsigned long CFG2_ADSREG2_MS  = 450000UL;
const unsigned long CFG2_XTRREGEN_MS = 150000UL;
const unsigned long CFG2_WAIT_MS     = 10000UL;
const unsigned long CFG2_RMMTFILL_MS = 25000UL;

// -----------------------------------------------------------------------------
// Global state
// -----------------------------------------------------------------------------

RoomMode      currentMode      = RoomMode::Idle;
Step          currentStep      = Step::None;
unsigned long stepStartMs      = 0;
unsigned long stepDurationMs   = 0;
unsigned long pendingAerationPart2Time = 0;

// Calibration Timing Block v1 runtime state (code addresses HR189..HR198 = external HR190..HR199)
static uint16_t calTimingSeq = 0;
static uint32_t calTimingCycleId = 0;
static uint32_t calTimingDurationS = (CALIBRATION_TIME_MS / 1000UL);
static uint32_t calTimingRemainingS = 0;
static uint32_t calTimingLastUpdateMs = 0;

// USER button no longer used; HMI triggers measurement via Modbus coil
// bool lastUserButtonState = HIGH;

// -----------------------------------------------------------------------------
// Modbus TCP configuration
// -----------------------------------------------------------------------------

// Static IP configuration for Opta
// IMPORTANT: MAC must be unique on the network (not the same as GIGA shield)
byte optaMac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x24, 0x9B };
IPAddress optaIp(192, 168, 137, 199);
IPAddress optaDns(192, 168, 137, 1);
IPAddress optaGateway(192, 168, 137, 1);
IPAddress optaSubnet(255, 255, 255, 0);

// Simple Modbus addressing (1-based for documentation, 0-based in code)

// -----------------------------------------------------------------------------
// Modbus register map (Holding Registers, 0-based addresses)
//
// RO (read-only; owned by Opta firmware, client writes are reverted):
//  - HR5..6   (u32):   Firmware build identifier split across two u16 registers
//                      HR5 = low 16 bits, HR6 = high 16 bits
//                      (for confirming correct firmware is running)
//  - HR7..9   (u16):   Semantic firmware version (for UI display)
//                      HR7 = major, HR8 = minor, HR9 = patch
//  - HR10..11  (%*100): O2/CO2 measured values
//  - HR19..20  (u16):   Room mode / Step
//  - HR60..67  (diag):  A0602 raw/mA/%/mode diagnostics
//  - HR70..71  (diag):  CO2 quick mismatch checks
//  - HR180     (u16):   outputs_mask (bit0 D0, bit1 D1, bit2 D2, bit3 D3,
//                       bit4 R1, bit5 R2, bit6 R3, bit7 R4, bit8 R5, bit9 R6,
//                       bit10 R7, bit11 R8)
//  - HR181     (u16):   alarms_mask raw digital input snapshot
//                       bit0 A0 analyzer_connected
//                       bit1 A1 flow_alarm
//                       bit2 A2 pressure_scrubber_right
//                       bit3 A3 pressure_scrubber_left
//                       bit4 A4 thermal_contacts_fans_1_2
//                       bit5 A5 compressed_air_alarm
//                       bit6 A6 transformer_alarm
//  - HR190..199 (Timing Block v1: Calibration; code addresses HR189..HR198)
//      HR190     (u16): version (=1)
//      HR191     (u16): seq
//      HR192     (u16): seq2
//      HR193     (u16): flags (bit0 SUPPORTED, bit1 VALID, bit2 DURATION_LATCHED)
//      HR194..195(u32): cycle_id (lo/hi)
//      HR196..197(u32): duration_s (lo/hi)
//      HR198..199(u32): remaining_s (lo/hi)
//
// RW (read/write; client may write, Opta clamps/applies when commanded):
//  - HR30..35  (scaled): O2 calibration/config
//  - HR40..45  (scaled): CO2 calibration/config
//  - HR52      (u16 ms): sweep relay ON dwell time (clamped 100..60000)
//  - HR100..101 (u16):   Control ownership arbitration
//                      HR100 = CONTROL_OWNER (0=NONE, 1=HMI, 2=PI)
//                      HR101 = CONTROL_CMD_ID (caller-provided u16 tag; Opta may overwrite)
//                      HR102..109 reserved for future metadata (reason/timeouts/etc)
//
// CMD (command semantics; write triggers action, value is cleared by firmware):
//  - HR50: HR_CAL_CMD (1=apply, 2=save stub, 3=restore defaults)
//  - HR51: HR_CAL_STATUS (0=idle/OK, 1=applied OK, 2=saved OK, 3=defaults restored OK,
//                          100+=error codes: 101 invalid cmd, 110 save failed, 111 load failed, 120 CRC mismatch)
// -----------------------------------------------------------------------------

// Coils
const uint16_t COIL_CMD_MEASURE_START   = 0; // 00001
const uint16_t COIL_CMD_CALIBRATE_START = 1; // 00002
const uint16_t COIL_CMD_SCRUB_CFG1_START = 2; // 00003 (new)
const uint16_t COIL_CMD_SCRUB_CFG2_START = 3; // 00004 (new) 
const uint16_t COIL_CMD_RELAY_TEST       = 4; // 00005 (start/stop relay sweep)


// Holding registers
const uint16_t HR_FW_BUILD_ID_LO = 5; // firmware build identifier low 16 bits
const uint16_t HR_FW_BUILD_ID_HI = 6; // firmware build identifier high 16 bits
const uint16_t HR_FW_VERSION_MAJOR = 7; // semantic version major
const uint16_t HR_FW_VERSION_MINOR = 8; // semantic version minor
const uint16_t HR_FW_VERSION_PATCH = 9; // semantic version patch
const uint16_t HR_O2_MEAS   = 10; // 40011 (% * 100)
const uint16_t HR_CO2_MEAS  = 11; // 40012 (% * 100)
const uint16_t HR_ROOM_MODE = 19; // 40020
const uint16_t HR_STEP      = 20; // 40021
const uint16_t HR_OUTPUTS_MASK = 180; // outputs bitmask snapshot
const uint16_t HR_ALARMS_MASK = 181; // raw digital input alarm bitmask snapshot
const uint16_t HR_CAL_TB_VERSION        = 189; // calibration timing block version (external HR190)
const uint16_t HR_CAL_TB_SEQ            = 190; // calibration timing block seq (external HR191)
const uint16_t HR_CAL_TB_SEQ2           = 191; // calibration timing block seq2 (external HR192)
const uint16_t HR_CAL_TB_FLAGS          = 192; // calibration timing block flags (external HR193)
const uint16_t HR_CAL_TB_CYCLE_ID_LO    = 193; // calibration timing block cycle_id low (external HR194)
const uint16_t HR_CAL_TB_CYCLE_ID_HI    = 194; // calibration timing block cycle_id high (external HR195)
const uint16_t HR_CAL_TB_DURATION_S_LO  = 195; // calibration timing block duration_s low (external HR196)
const uint16_t HR_CAL_TB_DURATION_S_HI  = 196; // calibration timing block duration_s high (external HR197)
const uint16_t HR_CAL_TB_REMAINING_S_LO = 197; // calibration timing block remaining_s low (external HR198)
const uint16_t HR_CAL_TB_REMAINING_S_HI = 198; // calibration timing block remaining_s high (external HR199)

static const uint16_t CAL_TB_FLAG_SUPPORTED        = (1u << 0);
static const uint16_t CAL_TB_FLAG_VALID            = (1u << 1);
static const uint16_t CAL_TB_FLAG_DURATION_LATCHED = (1u << 2);

// Calibration (setpoints) registers (Pi writes, Opta clamps/applies)
// Values are scaled integers:
//  - % values: % * 100
//  - alpha: alpha * 1000
//  - offsets: signed int16 stored in uint16 (% * 100)
//  - current: mA * 100
const uint16_t HR_O2_CAL_MIN_PCT_X100   = 30; // O2 min % at 4 mA
const uint16_t HR_O2_CAL_MAX_PCT_X100   = 31; // O2 max % at 20 mA
const uint16_t HR_O2_CAL_OFFSET_X100_S  = 32; // O2 offset % (signed)
const uint16_t HR_O2_CAL_ALPHA_X1000    = 33; // O2 LPF alpha
const uint16_t HR_O2_CAL_I_MIN_MA_X100   = 34; // O2 current min (mA): 4 for 4–20, 0 for 0–20
const uint16_t HR_O2_CAL_I_MAX_MA_X100   = 35; // O2 current max (mA): typically 20

const uint16_t HR_CO2_CAL_MIN_PCT_X100  = 40; // CO2 min % at 4 mA
const uint16_t HR_CO2_CAL_MAX_PCT_X100  = 41; // CO2 max % at 20 mA
const uint16_t HR_CO2_CAL_OFFSET_X100_S = 42; // CO2 offset % (signed)
const uint16_t HR_CO2_CAL_ALPHA_X1000   = 43; // CO2 LPF alpha
const uint16_t HR_CO2_CAL_I_MIN_MA_X100  = 44; // CO2 current min (mA): 4 for 4–20, 0 for 0–20
const uint16_t HR_CO2_CAL_I_MAX_MA_X100  = 45; // CO2 current max (mA): typically 20

const uint16_t HR_CAL_CMD               = 50; // 1=apply (RAM), 2=save (NVM), 3=restore defaults
const uint16_t HR_CAL_STATUS            = 51; // status/error codes (see register map)
const uint16_t HR_SWEEP_RELAY_ON_MS     = 52; // sweep relay ON dwell time (ms)

// Diagnostics registers (Opta writes, HMI can read)
// Reserved block: HR60..HR71
// Updated at <= 1 Hz to avoid any impact on Modbus servicing/UI responsiveness.
// Values are scaled integers:
//  - raw ADC: native count from A0602
//  - mA: mA * 100
//  - pct: % * 100 (pre-LPF)
//  - mode: 0=unknown, 1=current, 2=voltage
const uint16_t HR_A0602_O2_RAW_ADC       = 60;
const uint16_t HR_A0602_O2_MA_X100       = 61;
const uint16_t HR_A0602_O2_PCT_X100      = 62;
const uint16_t HR_A0602_O2_MODE          = 63;
const uint16_t HR_A0602_CO2_RAW_ADC      = 64;
const uint16_t HR_A0602_CO2_MA_X100      = 65;
const uint16_t HR_A0602_CO2_PCT_X100     = 66;
const uint16_t HR_A0602_CO2_MODE         = 67;

// CO2 quick-mismatch checks (based on measured mA but different current-range assumptions)
const uint16_t HR_A0602_CO2_PCT_IF_4_20_X100 = 70;
const uint16_t HR_A0602_CO2_PCT_IF_0_20_X100 = 71;

// -----------------------------------------------------------------------------
// Control ownership arbitration (reserved block: HR100..HR109)
//
// Purpose: provide an authoritative, PLC-side ownership indicator so HMI and Pi
// can coordinate "who is in control" (Option B later).
//
// NOTE: This does NOT change any existing register addresses.
//
// HR100 CONTROL_OWNER (RW)
//   0 = NONE
//   1 = HMI
//   2 = PI
// HR101 CONTROL_CMD_ID (RW)
//   Caller sets a unique u16 tag when requesting/taking ownership; Opta may
//   overwrite (e.g., to confirm the accepted value).
// HR102..HR109 reserved for future metadata:
//   HR102 CONTROL_LAST_CHANGE_REASON (RO): 0=boot,1=user_takeover,2=forced,3=timeout
//   HR103 CONTROL_LAST_CHANGE_UNIX_S_LO (RO) [reserved]
//   HR104 CONTROL_LAST_CHANGE_UNIX_S_HI (RO) [reserved]
//   HR105..HR109 [reserved]
// -----------------------------------------------------------------------------
const uint16_t HR_CONTROL_OWNER                 = 100;
const uint16_t HR_CONTROL_CMD_ID                = 101;
const uint16_t HR_CONTROL_LAST_CHANGE_REASON    = 102;
const uint16_t HR_CONTROL_LAST_CHANGE_UNIX_S_LO = 103;
const uint16_t HR_CONTROL_LAST_CHANGE_UNIX_S_HI = 104;

const uint16_t HR_HMI_HEARTBEAT                 = 105; // HMI heartbeat (RW u16; HMI increments)

static const uint32_t HMI_LEASE_TIMEOUT_MS = 5000u; // 5s: if no heartbeat changes, HMI ownership times out
static uint16_t hmiHeartbeatLastValue = 0;
static uint32_t hmiHeartbeatLastChangeMs = 0;

static const uint16_t CONTROL_OWNER_NONE = 0;
static const uint16_t CONTROL_OWNER_HMI  = 1;
static const uint16_t CONTROL_OWNER_PI   = 2;

static const uint16_t CONTROL_REASON_BOOT         = 0;
static const uint16_t CONTROL_REASON_USER_TAKEOVER= 1;
static const uint16_t CONTROL_REASON_FORCED       = 2;
static const uint16_t CONTROL_REASON_TIMEOUT      = 3;

// Runtime state (authoritative in firmware). Mirrors into HR100/HR101.
static uint16_t controlOwner = CONTROL_OWNER_HMI;
static uint16_t controlCmdId = 0;
static uint16_t controlLastChangeReason = CONTROL_REASON_BOOT;

#if USE_ARDUINO_MODBUS
// Deferred sync of control regs back into the Modbus mapping.
// We avoid holdingRegisterWrite() in the same loop() iteration as poll().
static bool mbControlSyncPending = false;
static uint32_t mbControlSyncArmedAtPollCount = 0;
static uint16_t mbControlSyncOwner = CONTROL_OWNER_HMI;
static uint16_t mbControlSyncCmdId = 0;
static uint16_t mbControlSyncReason = CONTROL_REASON_BOOT;
#endif

static inline uint16_t clampControlOwner(uint16_t v) {
  if (v == CONTROL_OWNER_NONE || v == CONTROL_OWNER_HMI || v == CONTROL_OWNER_PI) return v;
  return CONTROL_OWNER_NONE;
}

#if USE_ARDUINO_MODBUS
static inline void mb_arm_control_sync(uint16_t owner, uint16_t cmdId, uint16_t reason) {
  mbControlSyncPending = true;
  mbControlSyncArmedAtPollCount = mbPollCount;
  mbControlSyncOwner = owner;
  mbControlSyncCmdId = cmdId;
  mbControlSyncReason = reason;
}

static inline void mb_apply_control_sync_now() {
  if (!mbControlSyncPending) return;
  mb_write(HR_CONTROL_OWNER, mbControlSyncOwner);
  mb_write(HR_CONTROL_CMD_ID, mbControlSyncCmdId);
  mb_write(HR_CONTROL_LAST_CHANGE_REASON, mbControlSyncReason);
  // Time registers reserved (no RTC); keep at 0 for now.
  mb_write(HR_CONTROL_LAST_CHANGE_UNIX_S_LO, 0);
  mb_write(HR_CONTROL_LAST_CHANGE_UNIX_S_HI, 0);
  mbControlSyncPending = false;
}

static inline void mb_apply_control_sync_if_armed() {
  if (!mbControlSyncPending) return;
  if (mbPollCount > mbControlSyncArmedAtPollCount) {
    mb_apply_control_sync_now();
  }
}
#endif

static inline void setControlOwner(uint16_t newOwner, uint16_t reason, uint16_t cmdId) {
  // Update firmware-authoritative state.
  controlOwner = clampControlOwner(newOwner);
  controlCmdId = cmdId;
  controlLastChangeReason = reason;

#if USE_ARDUINO_MODBUS
  // Defer register writes to a safe loop() iteration.
  mb_arm_control_sync(controlOwner, controlCmdId, controlLastChangeReason);
#else
  // Legacy mode: write immediately into mirror only.
  holdingRegs[HR_CONTROL_OWNER] = controlOwner;
  holdingRegs[HR_CONTROL_CMD_ID] = controlCmdId;
  holdingRegs[HR_CONTROL_LAST_CHANGE_REASON] = controlLastChangeReason;
#endif
}

static const uint16_t HR_DIAG_NA_U16     = 0xFFFF;
static const uint16_t A0602_MODE_UNKNOWN = 0;
static const uint16_t A0602_MODE_CURRENT = 1;
static const uint16_t A0602_MODE_VOLTAGE = 2;

// Indices of the connected expansions on the Opta bus
int8_t analogExpIndex = -1;  // A0602
int8_t digitalExpIndex = -1; // D1608E

// AO602 physical channel mapping
// The O2 and CO2 4–20 mA loops are wired backwards on the AO602.
// Keep HR10 as O2 and HR11 as CO2 by mapping logical sensors to physical channels here.
static const uint8_t A0602_PHYS_CH_O2  = 1; // O2 transmitter is physically wired to AO602 ch1
static const uint8_t A0602_PHYS_CH_CO2 = 0; // CO2 transmitter is physically wired to AO602 ch0

// -----------------------------------------------------------------------------
// Placeholder APIs for analog inputs and extension relays
// -----------------------------------------------------------------------------

// 4-20 mA to engineering units
// Set to false to use real A0602 inputs instead of a sawtooth simulation
static bool SIMULATE_SENSORS = false;

// Simple calibration/scaling for sensors (percent ranges)
// Adjust these to match the transmitter scaling.
static float CAL_O2_MIN_PCT  = 0.0f;   // O2 range min (% at 4 mA)
static float CAL_O2_MAX_PCT  = 25.0f;  // O2 range max (% at 20 mA) — typical 0..25%
static float CAL_CO2_MIN_PCT = 0.0f;   // CO2 range min (% at 4 mA)
static float CAL_CO2_MAX_PCT = 10.0f;  // CO2 range max (% at 20 mA) — adjust to sensor

// Current output mode assumptions for each transmitter (0–20 mA vs 4–20 mA).
// These are configurable via holding registers so we can diagnose/fix a mismatch without reflashing.
static float CAL_O2_I_MIN_MA  = 4.0f;
static float CAL_O2_I_MAX_MA  = 20.0f;
static float CAL_CO2_I_MIN_MA = 4.0f;
static float CAL_CO2_I_MAX_MA = 20.0f;

// Optional offset to fine tune calibration (additive in percent)
static float CAL_O2_OFFSET_PCT  = 0.0f;  // e.g., +0.5 to lift readings slightly
static float CAL_CO2_OFFSET_PCT = 0.0f;

// Lightweight low-pass filter for smoothing
static float O2_LPF_ALPHA  = 0.2f; // 0..1; higher = faster
static float CO2_LPF_ALPHA = 0.2f;
static float o2Filtered = NAN;
static float co2Filtered = NAN;
static bool o2LpfInitialized = false;
static bool co2LpfInitialized = false;

// Defaults for "restore defaults" command
static const float DEFAULT_CAL_O2_MIN_PCT     = 0.0f;
static const float DEFAULT_CAL_O2_MAX_PCT     = 25.0f;
static const float DEFAULT_CAL_O2_OFFSET_PCT  = 0.0f;
static const float DEFAULT_O2_LPF_ALPHA       = 0.2f;
static const float DEFAULT_CAL_O2_I_MIN_MA    = 4.0f;
static const float DEFAULT_CAL_O2_I_MAX_MA    = 20.0f;

static const float DEFAULT_CAL_CO2_MIN_PCT    = 0.0f;
static const float DEFAULT_CAL_CO2_MAX_PCT    = 10.0f;
static const float DEFAULT_CAL_CO2_OFFSET_PCT = 0.0f;
static const float DEFAULT_CO2_LPF_ALPHA      = 0.2f;
static const float DEFAULT_CAL_CO2_I_MIN_MA   = 4.0f;
static const float DEFAULT_CAL_CO2_I_MAX_MA   = 20.0f;

// TEMP/DEMO: CO2 software offset (remove after demo)
// Apply AFTER the existing sensor read + scaling + (optional) filtering.
// Requirement: subtract 8.3 from the scaled CO2 percentage, clamp at 0.1.
static const float TEMP_DEMO_CO2_OFFSET_PCT = 8.3f;
static const float TEMP_DEMO_CO2_MIN_PCT    = 0.1f;
static inline float applyTempDemoCo2OffsetPct(float co2PctScaled) {
  if (!isfinite(co2PctScaled)) co2PctScaled = 0.0f;
  float adjusted = co2PctScaled - TEMP_DEMO_CO2_OFFSET_PCT;
  // Clamp to requested floor, and keep a sane upper bound.
  return clampf(adjusted, TEMP_DEMO_CO2_MIN_PCT, 100.0f);
}

static inline uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) {
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (uint16_t)v;
}

static inline void mb_write_u32(uint16_t loAddr, uint32_t value) {
  mb_write(loAddr, (uint16_t)(value & 0xFFFFu));
  mb_write((uint16_t)(loAddr + 1u), (uint16_t)((value >> 16) & 0xFFFFu));
}

static inline void publishCalibrationTimingBlock(bool valid, bool durationLatched) {
  uint16_t flags = CAL_TB_FLAG_SUPPORTED;
  if (valid) flags |= CAL_TB_FLAG_VALID;
  if (durationLatched) flags |= CAL_TB_FLAG_DURATION_LATCHED;

  uint16_t seqOdd = (uint16_t)((calTimingSeq + 1u) | 1u);
  calTimingSeq = seqOdd;
  mb_write(HR_CAL_TB_SEQ, seqOdd);
  mb_write(HR_CAL_TB_SEQ2, seqOdd);

  mb_write(HR_CAL_TB_VERSION, 1u);
  mb_write(HR_CAL_TB_FLAGS, flags);
  mb_write_u32(HR_CAL_TB_CYCLE_ID_LO, calTimingCycleId);
  mb_write_u32(HR_CAL_TB_DURATION_S_LO, calTimingDurationS);
  mb_write_u32(HR_CAL_TB_REMAINING_S_LO, calTimingRemainingS);

  uint16_t seqEven = (uint16_t)(seqOdd + 1u);
  calTimingSeq = seqEven;
  mb_write(HR_CAL_TB_SEQ, seqEven);
  mb_write(HR_CAL_TB_SEQ2, seqEven);
}

static inline void setCalibrationTimingIdle() {
  calTimingRemainingS = 0;
  calTimingDurationS = (uint32_t)(CALIBRATION_TIME_MS / 1000UL);
  publishCalibrationTimingBlock(false, false);
}

static float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

static float mapCurrentmA(float mA, float inMin, float inMax, float outMin, float outMax) {
  // Linear map with clamped fraction.
  float denom = (inMax - inMin);
  float frac = (denom != 0.0f) ? ((mA - inMin) / denom) : 0.0f;
  if (!isfinite(frac)) frac = 0.0f;
  frac = clampf(frac, 0.0f, 1.0f);
  return outMin + frac * (outMax - outMin);
}

static float map4to20mA(float mA, float outMin, float outMax) {
  return mapCurrentmA(mA, 4.0f, 20.0f, outMin, outMax);
}

float readO2Percent() {
  // Reading + filtering is performed in the 1 Hz updater; this is a lightweight getter.
  return (isfinite(o2Filtered) ? o2Filtered : 0.0f);
}

float readCO2Percent() {
  // Reading + filtering is performed in the 1 Hz updater; this is a lightweight getter.
  float co2 = (isfinite(co2Filtered) ? co2Filtered : 0.0f);
  return applyTempDemoCo2OffsetPct(co2);
}

// Read raw sensor currents (mA). Returns NAN if unavailable.
// If updateInputs is true, calls updateAnalogInputs() before reading.
static float readSensorCurrentmA(uint8_t channel, bool updateInputs = true) {
  if (SIMULATE_SENSORS) {
    // Steady ambient simulation with tiny noise
    float noise = (float)(millis() % 1000) / 1000.0f; // 0..1
    if (channel == A0602_PHYS_CH_O2) {
      // O2 ~20.9% on 0..25% scale -> ~17.4 mA
      return 17.4f + (noise - 0.5f) * 0.05f;
    }
    // CO2 ~0.04% on 0..10% scale -> ~4.064 mA
    return 4.064f + (noise - 0.5f) * 0.02f;
  }

  #ifdef HAVE_OPTA_BLUE
  if (analogExpIndex < 0) return NAN;
  AnalogExpansion exp = OptaController.getExpansion(analogExpIndex);
  if (!exp) return NAN;
  if (updateInputs) exp.updateAnalogInputs();
  float mA = exp.pinCurrent(channel, false);
  if (!isfinite(mA) || mA < 0.0f) return NAN;
  return mA;
  #else
  (void)channel;
  return NAN;
  #endif
}

static inline uint16_t mb_read_u16(uint16_t addr) {
#if USE_ARDUINO_MODBUS
  long v = modbusTCPServer.holdingRegisterRead((int)addr);
  if (v < 0) return 0;
  return (uint16_t)v;
#else
  if (addr < HOLDING_REGS_SIZE) return holdingRegs[addr];
  return 0;
#endif
}

static inline int16_t mb_read_s16(uint16_t addr) {
  return (int16_t)mb_read_u16(addr);
}

// -----------------------------------------------------------------------------
// Calibration persistence (HR30–35 and HR40–45 raw register values)
//
// Stored format:
//  - versioned record + CRC32, storing raw uint16 register contents
//  - stored in the last flash sector (internal flash)
// -----------------------------------------------------------------------------

static const uint32_t CAL_STORE_MAGIC   = 0x53434C50u; // 'SCLP'
static const uint16_t CAL_STORE_VERSION = 1;

struct CalStoreRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint16_t regs[12]; // HR30..35 (6) + HR40..45 (6)
  uint32_t crc32;
};

static uint32_t crc32_compute(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint32_t)data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

static bool cal_flash_get_addr_size(FlashIAP& flash, uint32_t& outAddr, uint32_t& outSize) {
  uint32_t flashStart = flash.get_flash_start();
  uint32_t flashSize  = flash.get_flash_size();
  if (flashSize == 0) return false;

  uint32_t flashEnd = flashStart + flashSize;
  // Place record in the last sector.
  uint32_t probeAddr = flashEnd - 1;
  uint32_t sectorSize = flash.get_sector_size(probeAddr);
  if (sectorSize == 0) return false;
  uint32_t sectorBase = flashEnd - sectorSize;

  outAddr = sectorBase;
  outSize = sectorSize;
  return true;
}

static bool cal_store_save(uint16_t* outErr) {
  if (outErr) *outErr = 0;

  FlashIAP flash;
  if (flash.init() != 0) {
    if (outErr) *outErr = 110;
    return false;
  }

  uint32_t addr = 0, sectorSize = 0;
  if (!cal_flash_get_addr_size(flash, addr, sectorSize)) {
    flash.deinit();
    if (outErr) *outErr = 110;
    return false;
  }

  CalStoreRecord rec{};
  rec.magic = CAL_STORE_MAGIC;
  rec.version = CAL_STORE_VERSION;
  rec.reserved = 0;

  // Read raw register values directly from the Modbus holding register map.
  // (Do not rely on holdingRegs[] mirror for RW registers.)
  rec.regs[0]  = mb_read_u16(HR_O2_CAL_MIN_PCT_X100);
  rec.regs[1]  = mb_read_u16(HR_O2_CAL_MAX_PCT_X100);
  rec.regs[2]  = mb_read_u16(HR_O2_CAL_OFFSET_X100_S);
  rec.regs[3]  = mb_read_u16(HR_O2_CAL_ALPHA_X1000);
  rec.regs[4]  = mb_read_u16(HR_O2_CAL_I_MIN_MA_X100);
  rec.regs[5]  = mb_read_u16(HR_O2_CAL_I_MAX_MA_X100);
  rec.regs[6]  = mb_read_u16(HR_CO2_CAL_MIN_PCT_X100);
  rec.regs[7]  = mb_read_u16(HR_CO2_CAL_MAX_PCT_X100);
  rec.regs[8]  = mb_read_u16(HR_CO2_CAL_OFFSET_X100_S);
  rec.regs[9]  = mb_read_u16(HR_CO2_CAL_ALPHA_X1000);
  rec.regs[10] = mb_read_u16(HR_CO2_CAL_I_MIN_MA_X100);
  rec.regs[11] = mb_read_u16(HR_CO2_CAL_I_MAX_MA_X100);

  rec.crc32 = crc32_compute((const uint8_t*)&rec, sizeof(CalStoreRecord) - sizeof(uint32_t));

  // Erase + program one record at the start of the last sector.
  // Keep operation minimal (single sector, single write).
  int eraseRc = flash.erase(addr, sectorSize);
  if (eraseRc != 0) {
    flash.deinit();
    if (outErr) *outErr = 110;
    return false;
  }

  uint32_t progSize = flash.get_page_size();
  uint32_t writeSize = sizeof(CalStoreRecord);
  uint32_t paddedSize = (writeSize + progSize - 1u) / progSize * progSize;
  if (paddedSize > sectorSize) {
    flash.deinit();
    if (outErr) *outErr = 110;
    return false;
  }

  static uint8_t pageBuf[128];
  uint8_t* buf = pageBuf;
  bool bufIsDynamic = false;
  if (progSize > sizeof(pageBuf)) {
    buf = (uint8_t*)malloc(progSize);
    if (!buf) {
      flash.deinit();
      if (outErr) *outErr = 110;
      return false;
    }
    bufIsDynamic = true;
  }

  // Program in page-sized chunks.
  const uint8_t* src = (const uint8_t*)&rec;
  for (uint32_t off = 0; off < paddedSize; off += progSize) {
    memset(buf, 0xFF, progSize);
    uint32_t toCopy = progSize;
    if (off + toCopy > writeSize) {
      if (off >= writeSize) toCopy = 0;
      else toCopy = writeSize - off;
    }
    if (toCopy > 0) memcpy(buf, src + off, toCopy);
    int progRc = flash.program(buf, addr + off, progSize);
    if (progRc != 0) {
      if (bufIsDynamic) free(buf);
      flash.deinit();
      if (outErr) *outErr = 110;
      return false;
    }
  }

  if (bufIsDynamic) free(buf);

  flash.deinit();
  return true;
}

static bool cal_store_load(uint16_t* outErr) {
  if (outErr) *outErr = 0;

  FlashIAP flash;
  if (flash.init() != 0) {
    if (outErr) *outErr = 111;
    return false;
  }

  uint32_t addr = 0, sectorSize = 0;
  if (!cal_flash_get_addr_size(flash, addr, sectorSize)) {
    flash.deinit();
    if (outErr) *outErr = 111;
    return false;
  }

  CalStoreRecord rec{};
  int readRc = flash.read(&rec, addr, sizeof(CalStoreRecord));
  flash.deinit();
  if (readRc != 0) {
    if (outErr) *outErr = 111;
    return false;
  }

  if (rec.magic != CAL_STORE_MAGIC || rec.version != CAL_STORE_VERSION) {
    // No valid saved record; treat as "missing" and keep defaults.
    return false;
  }

  uint32_t expected = crc32_compute((const uint8_t*)&rec, sizeof(CalStoreRecord) - sizeof(uint32_t));
  if (expected != rec.crc32) {
    if (outErr) *outErr = 120;
    return false;
  }

  // Copy raw saved values into holding registers and apply to runtime calibration.
  mb_write(HR_O2_CAL_MIN_PCT_X100,   rec.regs[0]);
  mb_write(HR_O2_CAL_MAX_PCT_X100,   rec.regs[1]);
  mb_write(HR_O2_CAL_OFFSET_X100_S,  rec.regs[2]);
  mb_write(HR_O2_CAL_ALPHA_X1000,    rec.regs[3]);
  mb_write(HR_O2_CAL_I_MIN_MA_X100,  rec.regs[4]);
  mb_write(HR_O2_CAL_I_MAX_MA_X100,  rec.regs[5]);
  mb_write(HR_CO2_CAL_MIN_PCT_X100,  rec.regs[6]);
  mb_write(HR_CO2_CAL_MAX_PCT_X100,  rec.regs[7]);
  mb_write(HR_CO2_CAL_OFFSET_X100_S, rec.regs[8]);
  mb_write(HR_CO2_CAL_ALPHA_X1000,   rec.regs[9]);
  mb_write(HR_CO2_CAL_I_MIN_MA_X100, rec.regs[10]);
  mb_write(HR_CO2_CAL_I_MAX_MA_X100, rec.regs[11]);

  mb_apply_calibration_from_registers();
  return true;
}

static void mb_sync_calibration_to_registers() {
  mb_write(HR_O2_CAL_MIN_PCT_X100,  (uint16_t)clamp_u16((int32_t)lroundf(CAL_O2_MIN_PCT * 100.0f), 0, 5000));
  mb_write(HR_O2_CAL_MAX_PCT_X100,  (uint16_t)clamp_u16((int32_t)lroundf(CAL_O2_MAX_PCT * 100.0f), 0, 5000));
  mb_write(HR_O2_CAL_OFFSET_X100_S, (uint16_t)(int16_t)clampf((float)lroundf(CAL_O2_OFFSET_PCT * 100.0f), -30000.0f, 30000.0f));
  mb_write(HR_O2_CAL_ALPHA_X1000,   (uint16_t)clamp_u16((int32_t)lroundf(clampf(O2_LPF_ALPHA, 0.0f, 1.0f) * 1000.0f), 0, 1000));
  mb_write(HR_O2_CAL_I_MIN_MA_X100, (uint16_t)clamp_u16((int32_t)lroundf(clampf(CAL_O2_I_MIN_MA, 0.0f, 30.0f) * 100.0f), 0, 3000));
  mb_write(HR_O2_CAL_I_MAX_MA_X100, (uint16_t)clamp_u16((int32_t)lroundf(clampf(CAL_O2_I_MAX_MA, 0.0f, 30.0f) * 100.0f), 0, 3000));

  mb_write(HR_CO2_CAL_MIN_PCT_X100,  (uint16_t)clamp_u16((int32_t)lroundf(CAL_CO2_MIN_PCT * 100.0f), 0, 5000));
  mb_write(HR_CO2_CAL_MAX_PCT_X100,  (uint16_t)clamp_u16((int32_t)lroundf(CAL_CO2_MAX_PCT * 100.0f), 0, 5000));
  mb_write(HR_CO2_CAL_OFFSET_X100_S, (uint16_t)(int16_t)clampf((float)lroundf(CAL_CO2_OFFSET_PCT * 100.0f), -30000.0f, 30000.0f));
  mb_write(HR_CO2_CAL_ALPHA_X1000,   (uint16_t)clamp_u16((int32_t)lroundf(clampf(CO2_LPF_ALPHA, 0.0f, 1.0f) * 1000.0f), 0, 1000));
  mb_write(HR_CO2_CAL_I_MIN_MA_X100, (uint16_t)clamp_u16((int32_t)lroundf(clampf(CAL_CO2_I_MIN_MA, 0.0f, 30.0f) * 100.0f), 0, 3000));
  mb_write(HR_CO2_CAL_I_MAX_MA_X100, (uint16_t)clamp_u16((int32_t)lroundf(clampf(CAL_CO2_I_MAX_MA, 0.0f, 30.0f) * 100.0f), 0, 3000));
}

static void mb_apply_calibration_from_registers() {
  // Read requested values
  float o2Min = (float)mb_read_u16(HR_O2_CAL_MIN_PCT_X100) / 100.0f;
  float o2Max = (float)mb_read_u16(HR_O2_CAL_MAX_PCT_X100) / 100.0f;
  float o2Off = (float)mb_read_s16(HR_O2_CAL_OFFSET_X100_S) / 100.0f;
  float o2A   = (float)mb_read_u16(HR_O2_CAL_ALPHA_X1000) / 1000.0f;
  float o2Imin = (float)mb_read_u16(HR_O2_CAL_I_MIN_MA_X100) / 100.0f;
  float o2Imax = (float)mb_read_u16(HR_O2_CAL_I_MAX_MA_X100) / 100.0f;

  float c2Min = (float)mb_read_u16(HR_CO2_CAL_MIN_PCT_X100) / 100.0f;
  float c2Max = (float)mb_read_u16(HR_CO2_CAL_MAX_PCT_X100) / 100.0f;
  float c2Off = (float)mb_read_s16(HR_CO2_CAL_OFFSET_X100_S) / 100.0f;
  float c2A   = (float)mb_read_u16(HR_CO2_CAL_ALPHA_X1000) / 1000.0f;
  float c2Imin = (float)mb_read_u16(HR_CO2_CAL_I_MIN_MA_X100) / 100.0f;
  float c2Imax = (float)mb_read_u16(HR_CO2_CAL_I_MAX_MA_X100) / 100.0f;

  // Clamp to safe ranges
  o2Min = clampf(o2Min, 0.0f, 50.0f);
  o2Max = clampf(o2Max, 0.1f, 50.0f);
  if (o2Max <= o2Min) o2Max = o2Min + 0.1f;
  o2Off = clampf(o2Off, -10.0f, 10.0f);
  o2A   = clampf(o2A, 0.0f, 1.0f);

  o2Imin = clampf(o2Imin, 0.0f, 30.0f);
  o2Imax = clampf(o2Imax, 0.0f, 30.0f);
  if (o2Imax <= (o2Imin + 0.01f)) {
    // Invalid current range; fall back to standard 4–20 mA.
    o2Imin = 4.0f;
    o2Imax = 20.0f;
  }

  c2Min = clampf(c2Min, 0.0f, 50.0f);
  c2Max = clampf(c2Max, 0.1f, 50.0f);
  if (c2Max <= c2Min) c2Max = c2Min + 0.1f;
  c2Off = clampf(c2Off, -10.0f, 10.0f);
  c2A   = clampf(c2A, 0.0f, 1.0f);

  c2Imin = clampf(c2Imin, 0.0f, 30.0f);
  c2Imax = clampf(c2Imax, 0.0f, 30.0f);
  if (c2Imax <= (c2Imin + 0.01f)) {
    c2Imin = 4.0f;
    c2Imax = 20.0f;
  }

  // Apply
  CAL_O2_MIN_PCT = o2Min;
  CAL_O2_MAX_PCT = o2Max;
  CAL_O2_OFFSET_PCT = o2Off;
  O2_LPF_ALPHA = o2A;
  CAL_O2_I_MIN_MA = o2Imin;
  CAL_O2_I_MAX_MA = o2Imax;

  CAL_CO2_MIN_PCT = c2Min;
  CAL_CO2_MAX_PCT = c2Max;
  CAL_CO2_OFFSET_PCT = c2Off;
  CO2_LPF_ALPHA = c2A;
  CAL_CO2_I_MIN_MA = c2Imin;
  CAL_CO2_I_MAX_MA = c2Imax;

  // Write accepted values back
  mb_sync_calibration_to_registers();
}

static void mb_restore_calibration_defaults() {
  CAL_O2_MIN_PCT = DEFAULT_CAL_O2_MIN_PCT;
  CAL_O2_MAX_PCT = DEFAULT_CAL_O2_MAX_PCT;
  CAL_O2_OFFSET_PCT = DEFAULT_CAL_O2_OFFSET_PCT;
  O2_LPF_ALPHA = DEFAULT_O2_LPF_ALPHA;
  CAL_O2_I_MIN_MA = DEFAULT_CAL_O2_I_MIN_MA;
  CAL_O2_I_MAX_MA = DEFAULT_CAL_O2_I_MAX_MA;

  CAL_CO2_MIN_PCT = DEFAULT_CAL_CO2_MIN_PCT;
  CAL_CO2_MAX_PCT = DEFAULT_CAL_CO2_MAX_PCT;
  CAL_CO2_OFFSET_PCT = DEFAULT_CAL_CO2_OFFSET_PCT;
  CO2_LPF_ALPHA = DEFAULT_CO2_LPF_ALPHA;
  CAL_CO2_I_MIN_MA = DEFAULT_CAL_CO2_I_MIN_MA;
  CAL_CO2_I_MAX_MA = DEFAULT_CAL_CO2_I_MAX_MA;

  // Reset filters so we don't ramp from stale values
  o2Filtered = NAN;
  co2Filtered = NAN;
  o2LpfInitialized = false;
  co2LpfInitialized = false;

  mb_sync_calibration_to_registers();
}

static void mb_service_calibration_command() {
  uint16_t cmd = mb_read_u16(HR_CAL_CMD);
  if (cmd == 0) return;

  if (cmd == 1) {
    mb_apply_calibration_from_registers();
    mb_write(HR_CAL_STATUS, 1);
  } else if (cmd == 2) {
    uint16_t err = 0;
    bool ok = cal_store_save(&err);
    if (ok) {
      mb_write(HR_CAL_STATUS, 2);
    } else {
      // Save failed
      mb_write(HR_CAL_STATUS, (err >= 100) ? err : 110);
    }
  } else if (cmd == 3) {
    mb_restore_calibration_defaults();
    mb_write(HR_CAL_STATUS, 3);
  } else {
    // Unknown command
    mb_write(HR_CAL_STATUS, 101);
  }

  // Clear command register after handling
  mb_write(HR_CAL_CMD, 0);
}

static void updateA0602Diagnostics1Hz(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  // Diagnostics update rate is capped at 1 Hz.
  // This is intentionally slower than the 2 Hz live values so it remains low-impact.
  if ((uint32_t)(nowMs - lastMs) < 1000u) return;
  lastMs = nowMs;

  // Default all diagnostics to NA until we successfully sample.
  mb_write(HR_A0602_O2_RAW_ADC,   HR_DIAG_NA_U16);
  mb_write(HR_A0602_O2_MA_X100,   HR_DIAG_NA_U16);
  mb_write(HR_A0602_O2_PCT_X100,  HR_DIAG_NA_U16);
  mb_write(HR_A0602_O2_MODE,      A0602_MODE_UNKNOWN);
  mb_write(HR_A0602_CO2_RAW_ADC,  HR_DIAG_NA_U16);
  mb_write(HR_A0602_CO2_MA_X100,  HR_DIAG_NA_U16);
  mb_write(HR_A0602_CO2_PCT_X100, HR_DIAG_NA_U16);
  mb_write(HR_A0602_CO2_MODE,     A0602_MODE_UNKNOWN);
  mb_write(HR_A0602_CO2_PCT_IF_4_20_X100, HR_DIAG_NA_U16);
  mb_write(HR_A0602_CO2_PCT_IF_0_20_X100, HR_DIAG_NA_U16);

  #ifdef HAVE_OPTA_BLUE
  if (analogExpIndex < 0) {
    if (LOG_A0602_DIAG && Serial) Serial.println("[A0602] not detected");
    return;
  }

  AnalogExpansion ae = OptaController.getExpansion(analogExpIndex);
  if (!ae) {
    if (LOG_A0602_DIAG && Serial) Serial.println("[A0602] getExpansion failed");
    return;
  }

  // No extra I/O here: we read the most recently sampled A0602 values.
  // `updateSensors2Hz()` already refreshes A0602 via `updateAnalogInputs()`.

  for (uint8_t ch = 0; ch < 2; ch++) {
    bool isCurrent = ae.isChCurrentAdc(ch, false);
    bool isVoltage = ae.isChVoltageAdc(ch, false);
    uint16_t modeCode = isCurrent ? A0602_MODE_CURRENT : (isVoltage ? A0602_MODE_VOLTAGE : A0602_MODE_UNKNOWN);
    const char* mode = (modeCode == A0602_MODE_CURRENT) ? "current" : ((modeCode == A0602_MODE_VOLTAGE) ? "voltage" : "unknown");
    uint16_t raw = ae.getAdc(ch, false);

    // mA is only meaningful if the channel is truly configured as current ADC.
    float mA = ae.pinCurrent(ch, false);
    float v = ae.pinVoltage(ch, false);

    float pct = 0.0f;
    bool havePct = false;
    if (isCurrent && isfinite(mA)) {
      if (ch == A0602_PHYS_CH_O2) {
        pct = mapCurrentmA(mA, CAL_O2_I_MIN_MA, CAL_O2_I_MAX_MA, CAL_O2_MIN_PCT, CAL_O2_MAX_PCT) + CAL_O2_OFFSET_PCT;
      } else if (ch == A0602_PHYS_CH_CO2) {
        pct = mapCurrentmA(mA, CAL_CO2_I_MIN_MA, CAL_CO2_I_MAX_MA, CAL_CO2_MIN_PCT, CAL_CO2_MAX_PCT) + CAL_CO2_OFFSET_PCT;
        // TEMP/DEMO: apply fixed offset AFTER scaling.
        pct = applyTempDemoCo2OffsetPct(pct);
      }
      // Note: CO2 is already clamped by applyTempDemoCo2OffsetPct().
      pct = clampf(pct, 0.0f, 100.0f);
      havePct = true;
    }

    // CO2 mismatch check: compute expected % for two common current modes.
    float co2If4to20 = 0.0f;
    float co2If0to20 = 0.0f;
    bool haveCo2Compare = false;
    if (isCurrent && isfinite(mA) && (ch == A0602_PHYS_CH_CO2)) {
      co2If4to20 = mapCurrentmA(mA, 4.0f, 20.0f, CAL_CO2_MIN_PCT, CAL_CO2_MAX_PCT) + CAL_CO2_OFFSET_PCT;
      co2If0to20 = mapCurrentmA(mA, 0.0f, 20.0f, CAL_CO2_MIN_PCT, CAL_CO2_MAX_PCT) + CAL_CO2_OFFSET_PCT;
      // TEMP/DEMO: keep CO2 diagnostics consistent with the displayed/transmitted CO2.
      co2If4to20 = applyTempDemoCo2OffsetPct(co2If4to20);
      co2If0to20 = applyTempDemoCo2OffsetPct(co2If0to20);
      co2If4to20 = clampf(co2If4to20, 0.0f, 100.0f);
      co2If0to20 = clampf(co2If0to20, 0.0f, 100.0f);
      haveCo2Compare = true;
    }

    // Mirror diagnostics into holding registers (so the GIGA can display without Serial).
    if (ch == A0602_PHYS_CH_O2) {
      mb_write(HR_A0602_O2_RAW_ADC, raw);
      mb_write(HR_A0602_O2_MODE, modeCode);
      mb_write(HR_A0602_O2_MA_X100,  (isCurrent && isfinite(mA)) ? (uint16_t)clamp_u16((int32_t)lroundf(mA * 100.0f), 0, 65535) : HR_DIAG_NA_U16);
      mb_write(HR_A0602_O2_PCT_X100, (havePct) ? (uint16_t)clamp_u16((int32_t)lroundf(pct * 100.0f), 0, 10000) : HR_DIAG_NA_U16);
    } else if (ch == A0602_PHYS_CH_CO2) {
      mb_write(HR_A0602_CO2_RAW_ADC, raw);
      mb_write(HR_A0602_CO2_MODE, modeCode);
      mb_write(HR_A0602_CO2_MA_X100,  (isCurrent && isfinite(mA)) ? (uint16_t)clamp_u16((int32_t)lroundf(mA * 100.0f), 0, 65535) : HR_DIAG_NA_U16);
      mb_write(HR_A0602_CO2_PCT_X100, (havePct) ? (uint16_t)clamp_u16((int32_t)lroundf(pct * 100.0f), 0, 10000) : HR_DIAG_NA_U16);
      mb_write(HR_A0602_CO2_PCT_IF_4_20_X100, (haveCo2Compare) ? (uint16_t)clamp_u16((int32_t)lroundf(co2If4to20 * 100.0f), 0, 10000) : HR_DIAG_NA_U16);
      mb_write(HR_A0602_CO2_PCT_IF_0_20_X100, (haveCo2Compare) ? (uint16_t)clamp_u16((int32_t)lroundf(co2If0to20 * 100.0f), 0, 10000) : HR_DIAG_NA_U16);
    }

    if (LOG_A0602_DIAG && Serial) {
      // Example format requested (plus V=... when not in current mode)
      Serial.print("[A0602] ch");
      Serial.print((int)ch);
      Serial.print(" mode=");
      Serial.print(mode);
      Serial.print(" raw=");
      Serial.print(raw);
      Serial.print(" mA=");
      if (isfinite(mA)) Serial.print(mA, 2); else Serial.print("NA");
      if (!isCurrent) {
        Serial.print(" V=");
        if (isfinite(v)) Serial.print(v, 3); else Serial.print("NA");
      }
      Serial.print(" pct=");
      Serial.println(pct, 2);
    }
  }
  #endif
}

static void updateSensors2Hz(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  if ((uint32_t)(nowMs - lastMs) < 500u) return;
  lastMs = nowMs;

  // Publish live sensor values at 2 Hz for the HMI.
  // This is safe because it's local math + local holding-register writes only.
  // It does NOT change Modbus servicing/polling rate; network traffic remains client-driven.
  // One A0602 refresh per cycle: ch0 triggers updateAnalogInputs(); ch1 reuses cached values.
  float o2mA  = readSensorCurrentmA(A0602_PHYS_CH_O2, true);
  float co2mA = readSensorCurrentmA(A0602_PHYS_CH_CO2, false);

  float o2PctRaw = isfinite(o2mA)  ? (mapCurrentmA(o2mA,  CAL_O2_I_MIN_MA,  CAL_O2_I_MAX_MA,  CAL_O2_MIN_PCT,  CAL_O2_MAX_PCT)  + CAL_O2_OFFSET_PCT)  : 0.0f;
  float c2PctRaw = isfinite(co2mA) ? (mapCurrentmA(co2mA, CAL_CO2_I_MIN_MA, CAL_CO2_I_MAX_MA, CAL_CO2_MIN_PCT, CAL_CO2_MAX_PCT) + CAL_CO2_OFFSET_PCT) : 0.0f;

  o2PctRaw = clampf(o2PctRaw, 0.0f, 100.0f);
  c2PctRaw = clampf(c2PctRaw, 0.0f, 100.0f);

  // Initialize LPF on first sample to avoid startup ramp from 0.
  if (!o2LpfInitialized || !isfinite(o2Filtered)) {
    o2Filtered = o2PctRaw;
    o2LpfInitialized = true;
  } else {
    o2Filtered = (1.0f - O2_LPF_ALPHA) * o2Filtered + O2_LPF_ALPHA * o2PctRaw;
  }

  if (!co2LpfInitialized || !isfinite(co2Filtered)) {
    co2Filtered = c2PctRaw;
    co2LpfInitialized = true;
  } else {
    co2Filtered = (1.0f - CO2_LPF_ALPHA) * co2Filtered + CO2_LPF_ALPHA * c2PctRaw;
  }

  // Publish to holding regs with rounding
  float co2ForPublish = readCO2Percent(); // includes TEMP/DEMO offset + clamp
  uint16_t o2Scaled  = (uint16_t)clamp_u16((int32_t)lroundf(o2Filtered * 100.0f), 0, 65535);
  uint16_t co2Scaled = (uint16_t)clamp_u16((int32_t)lroundf(co2ForPublish * 100.0f), 0, 65535);
  mb_write(HR_O2_MEAS,  o2Scaled);
  mb_write(HR_CO2_MEAS, co2Scaled);
}

// Output state tracking for Modbus snapshot (do not rely on digitalRead)
static bool outputPinState[4] = { false, false, false, false }; // D0..D3
static bool relayState[9] = { false, false, false, false, false, false, false, false, false }; // 1..8

static inline uint16_t updateOutputsSnapshot() {
  uint16_t mask = 0;
  if (outputPinState[0]) mask |= (1u << 0);
  if (outputPinState[1]) mask |= (1u << 1);
  if (outputPinState[2]) mask |= (1u << 2);
  if (outputPinState[3]) mask |= (1u << 3);
  for (uint8_t i = 1; i <= 8; i++) {
    if (relayState[i]) mask |= (uint16_t)(1u << (i + 3)); // relay1 -> bit4
  }
  return mask;
}

static inline uint16_t updateAlarmsSnapshot() {
  // Export raw digitalRead states (no polarity inversion assumptions).
  uint16_t mask = 0;
  if (digitalRead(PIN_A_CON_ALA) == HIGH) mask |= (1u << 0);
  if (digitalRead(PIN_FLO_ALA) == HIGH) mask |= (1u << 1);
  if (digitalRead(PIN_R_C_P_ALA) == HIGH) mask |= (1u << 2);
  if (digitalRead(PIN_L_C_P_ALA) == HIGH) mask |= (1u << 3);
  if (digitalRead(PIN_F_O_ALA) == HIGH) mask |= (1u << 4);
  if (digitalRead(PIN_A_P_ALA) == HIGH) mask |= (1u << 5);
  if (digitalRead(PIN_TRA_ALA) == HIGH) mask |= (1u << 6);
  return mask;
}

static inline void publishSnapshotRegistersSafe(uint32_t nowMs) {
  // Snapshot register publishing is rate-limited and performed before poll().
  // This avoids writing holding registers in the post-poll response window.
  static uint32_t lastSnapshotMs = 0;
  static const uint32_t SNAPSHOT_PERIOD_MS = 100; // 10 Hz
  if (lastSnapshotMs != 0 && (uint32_t)(nowMs - lastSnapshotMs) < SNAPSHOT_PERIOD_MS) return;
  lastSnapshotMs = nowMs;

  mb_write(HR_OUTPUTS_MASK, updateOutputsSnapshot());
  mb_write(HR_ALARMS_MASK, updateAlarmsSnapshot());
}

static inline void setOutputPin(int pin, bool on) {
  digitalWrite(pin, on ? HIGH : LOW);
  if (pin == PIN_FAN_1) outputPinState[0] = on;
  else if (pin == PIN_FAN_2) outputPinState[1] = on;
  else if (pin == PIN_RM_V_1) outputPinState[2] = on;
  else if (pin == PIN_N2_RM_V_1) outputPinState[3] = on;
  updateOutputsSnapshot();
}

static inline void setRelayState(int relayIndex, bool on) {
  if (relayIndex >= 1 && relayIndex <= 8) {
    relayState[relayIndex] = on;
    updateOutputsSnapshot();
  }
}

// Drive D1608E relays via DigitalExpansion
void setRelay(int relayIndex, bool on) {
  #ifdef HAVE_OPTA_BLUE
  if (digitalExpIndex < 0) {
    Serial.println("[Relay] No digital expansion detected; skipping write");
    setRelayState(relayIndex, on);
    return;
  }
  ExpansionType_t type = OptaController.getExpansionType(digitalExpIndex);
  // Library channels are 0-based; REL_* constants are 1-based
  uint8_t ch = (relayIndex > 0) ? (uint8_t)(relayIndex - 1) : 0;
  if (type == EXPANSION_OPTA_DIGITAL_MEC) {
    DigitalMechExpansion mechExp = OptaController.getExpansion(digitalExpIndex);
    if (!mechExp) return;
    mechExp.digitalWrite(ch, on ? HIGH : LOW);
    mechExp.updateDigitalOutputs();
  } else if (type == EXPANSION_OPTA_DIGITAL_STS) {
    DigitalStSolidExpansion stsolidExp = OptaController.getExpansion(digitalExpIndex);
    if (!stsolidExp) return;
    stsolidExp.digitalWrite(ch, on ? HIGH : LOW);
    stsolidExp.updateDigitalOutputs();
  } else {
    Serial.println("[Relay] Unknown digital expansion type; skipping");
    return;
  }
  #else
  // Stub: mirror to LED only when library absent
  (void)relayIndex; (void)on;
  #endif

  // Debug: log relay switching action
  Serial.print("Relay ");
  Serial.print(relayIndex);
  Serial.print(" -> ");
  Serial.println(on ? "ON" : "OFF");

  // Mirror relay state to built-in LED
  mirrorRelayToLed(relayIndex, on);
  setRelayState(relayIndex, on);
}

// -----------------------------------------------------------------------------
// Output helpers for action groups
// -----------------------------------------------------------------------------

void setAllOutputsOff() {
  if (LOG_ACTIONS) Serial.println("Action: setAllOutputsOff");
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, LOW);
  setOutputPin(PIN_RM_V_1, LOW);
  setOutputPin(PIN_N2_RM_V_1, LOW);

  setRelay(REL_MEAS_P,     false);
  setRelay(REL_V_4_4,      false);
  setRelay(REL_V_2_3,      false);
  setRelay(REL_V_1_1,      false);
  setRelay(REL_M_V_N2_GEN, false);
  setRelay(REL_MEAS_V_CAL, false);
  setRelay(REL_MEAS_V_A_I, false);
  setRelay(REL_N2_REC_V,   false);
}

// -----------------------------------------------------------------------------
// Relay sequencer (D1608E) test logic inspired by Opta_MA_testing
// -----------------------------------------------------------------------------

static const uint16_t SWEEP_RELAY_ON_MS_DEFAULT = 2000;
static const uint16_t SWEEP_RELAY_ON_MS_MIN = 100;
static const uint16_t SWEEP_RELAY_ON_MS_MAX = 60000;

static inline uint16_t clampSweepRelayOnDurationMs(uint32_t valueMs) {
  if (valueMs < SWEEP_RELAY_ON_MS_MIN) return SWEEP_RELAY_ON_MS_MIN;
  if (valueMs > SWEEP_RELAY_ON_MS_MAX) return SWEEP_RELAY_ON_MS_MAX;
  return (uint16_t)valueMs;
}

static const uint8_t SWEEP_OUTPUT_COUNT = 12; // D0..D3, then Relay1..Relay8
struct RelaySweepState {
  bool sweep_active;
  uint8_t current_output_index; // 1..12
  uint32_t last_transition_ms;
  bool phase_on;
};

static RelaySweepState relaySweep = { false, 0, 0, false };
static uint16_t sweepRelayOnTimeMsLatched = SWEEP_RELAY_ON_MS_DEFAULT;

static void relaySweepSetRelay(uint8_t relayIndex1Based, bool on);

static void relaySweepSetOutput(uint8_t outputIndex1Based, bool on) {
  if (outputIndex1Based == 1) { setOutputPin(PIN_FAN_1, on); return; }      // D0
  if (outputIndex1Based == 2) { setOutputPin(PIN_FAN_2, on); return; }      // D1
  if (outputIndex1Based == 3) { setOutputPin(PIN_RM_V_1, on); return; }     // D2
  if (outputIndex1Based == 4) { setOutputPin(PIN_N2_RM_V_1, on); return; }  // D3
  if (outputIndex1Based >= 5 && outputIndex1Based <= SWEEP_OUTPUT_COUNT) {
    relaySweepSetRelay((uint8_t)(outputIndex1Based - 4), on); // relay1..relay8
  }
}

static void relaySweepSetRelay(uint8_t relayIndex1Based, bool on) {
  #ifdef HAVE_OPTA_BLUE
  if (digitalExpIndex < 0 || relayIndex1Based < 1 || relayIndex1Based > 8) {
    Serial.println("[RelaySeq] No digital expansion; skipping");
    setRelayState(relayIndex1Based, on);
    return;
  }
  ExpansionType_t type = OptaController.getExpansionType(digitalExpIndex);
  uint8_t ch = (uint8_t)(relayIndex1Based - 1);
  if (type == EXPANSION_OPTA_DIGITAL_MEC) {
    DigitalMechExpansion mechExp = OptaController.getExpansion(digitalExpIndex);
    if (!mechExp) return;
    mechExp.digitalWrite(ch, on ? HIGH : LOW);
    mechExp.updateDigitalOutputs();
  } else if (type == EXPANSION_OPTA_DIGITAL_STS) {
    DigitalStSolidExpansion stsolidExp = OptaController.getExpansion(digitalExpIndex);
    if (!stsolidExp) return;
    stsolidExp.digitalWrite(ch, on ? HIGH : LOW);
    stsolidExp.updateDigitalOutputs();
  } else {
    Serial.println("[RelaySeq] Unknown digital expansion type; skipping");
    return;
  }
  #else
  (void)relayIndex1Based; (void)on;
  #endif
  // Mirror to onboard LED for quick visual confirmation
  mirrorRelayToLed(relayIndex1Based, on);
  setRelayState(relayIndex1Based, on);
}

static void relaySweepSetAllOff() {
  for (uint8_t i = 1; i <= SWEEP_OUTPUT_COUNT; i++) {
    relaySweepSetOutput(i, false);
  }
}

static void relaySweepBegin() {
  // Sanity: ensure digital expansion present before attempting
  if (digitalExpIndex < 0) {
    Serial.println("[RelaySeq] Digital expansion not detected (index=-1). Check 24V/backplane.");
  }
  relaySweep.sweep_active = true;
  relaySweep.current_output_index = 1;
  relaySweep.phase_on = true;
  sweepRelayOnTimeMsLatched = clampSweepRelayOnDurationMs((uint32_t)holdingRegs[HR_SWEEP_RELAY_ON_MS]);
  relaySweep.last_transition_ms = millis();
  relaySweepSetAllOff();
  relaySweepSetOutput(relaySweep.current_output_index, true);
  if (LOG_SWEEP_EVENTS) Serial.println("Relay sequencer: start");
}

static void relaySweepStop() {
  relaySweepSetAllOff();
  relaySweep.sweep_active = false;
  relaySweep.current_output_index = 0;
  relaySweep.phase_on = false;
  if (LOG_SWEEP_EVENTS) Serial.println("Relay sequencer: finished");
}

static inline void relaySweepTick(uint32_t nowMs) {
  if (!relaySweep.sweep_active) return;

  if (relaySweep.phase_on) {
    if ((uint32_t)(nowMs - relaySweep.last_transition_ms) >= (uint32_t)sweepRelayOnTimeMsLatched) {
      relaySweepSetOutput(relaySweep.current_output_index, false);
      relaySweep.phase_on = false;
      relaySweep.last_transition_ms = nowMs;
    }
    return;
  }

  if (relaySweep.current_output_index < SWEEP_OUTPUT_COUNT) {
    relaySweep.current_output_index++;
    relaySweepSetOutput(relaySweep.current_output_index, true);
    relaySweep.phase_on = true;
    relaySweep.last_transition_ms = nowMs;
  } else {
    relaySweepStop();
  }
}

// Measurement cycle: Fan_1, Fan_2, V_2_3, V_4_4, RM_V_1, Meas_P, Meas_V_A_I
void applyMeasurementOutputs() {
  if (LOG_ACTIONS) Serial.println("Action: applyMeasurementOutputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_2_3,      true);
  setRelay(REL_V_4_4,      true);
  setRelay(REL_MEAS_P,     true);
  setRelay(REL_MEAS_V_A_I, true);
}

// Aeration part 1: Fan_2, V_1_1, V_2_3, V_4_4, RM_V_1
void applyAerationPart1Outputs() {
  Serial.println("Action: applyAerationPart1Outputs");
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_1_1, true);
  setRelay(REL_V_2_3, true);
  setRelay(REL_V_4_4, true);
}

// Aeration part 2: Fan_1, Fan_2, V_1_1, V_2_3, V_4_4, RM_V_1
void applyAerationPart2Outputs() {
  Serial.println("Action: applyAerationPart2Outputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_1_1, true);
  setRelay(REL_V_2_3, true);
  setRelay(REL_V_4_4, true);
}

// Config 1 - Ads/Reg part 1: Fan_1, Fan_2, V_2_3, V_4_4, RM_V_1, Meas_P, Meas_V_A_O
void applyCfg1_AdsReg1Outputs() {
  Serial.println("Action: applyCfg1_AdsReg1Outputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_2_3,      true);
  setRelay(REL_V_4_4,      true);
  setRelay(REL_MEAS_P,     true);
  // Removed: calibration valve not used in scrub ads/reg sequences
}

// Config 1 - Ads/Reg part 2: Fan_1, Fan_2, V_2_3, V_4_4, RM_V_1
void applyCfg1_AdsReg2Outputs() {
  Serial.println("Action: applyCfg1_AdsReg2Outputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_2_3, true);
  setRelay(REL_V_4_4, true);
}

// Config 1 - Extra Regen Cylinder #1: Fan_2, V_2_3
void applyCfg1_XtrRegenOutputs() {
  Serial.println("Action: applyCfg1_XtrRegenOutputs");
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, HIGH);
  setRelay(REL_V_2_3, true);
  setRelay(REL_V_4_4, false);
}

// Config 1 - Wait: V_4_4
void applyCfg1_WaitOutputs() {
  Serial.println("Action: applyCfg1_WaitOutputs");
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, LOW);
  setRelay(REL_V_4_4, true);
}

// Config 1 - Room MT/Fill: Fan_1, Fan_2, V_2_3, RM_V_1
void applyCfg1_RmMtFillOutputs() {
  Serial.println("Action: applyCfg1_RmMtFillOutputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_2_3, true);
}

// Config 2 - Ads/Reg part 1: Fan_1, Fan_2, V_1_1, RM_V_1, Meas_P, Meas_V_A_O
void applyCfg2_AdsReg1Outputs() {
  Serial.println("Action: applyCfg2_AdsReg1Outputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_1_1,      true);
  setRelay(REL_MEAS_P,     true);
  // Removed: calibration valve not used in scrub ads/reg sequences
}

// Config 2 - Ads/Reg part 2: Fan_1, Fan_2, V_1_1, RM_V_1
void applyCfg2_AdsReg2Outputs() {
  Serial.println("Action: applyCfg2_AdsReg2Outputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_1_1, true);
}

// Config 2 - Extra Regen Cylinder #1: Fan_2, V_1_1
void applyCfg2_XtrRegenOutputs() {
  Serial.println("Action: applyCfg2_XtrRegenOutputs");
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, HIGH);
  setRelay(REL_V_1_1, true);
}

// Config 2 - Wait: V_1_1
void applyCfg2_WaitOutputs() {
  Serial.println("Action: applyCfg2_WaitOutputs");
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, LOW);
  setRelay(REL_V_1_1, true);
}

// Config 2 - Room MT/Fill: Fan_1, Fan_2, V_2_3, RM_V_1
void applyCfg2_RmMtFillOutputs() {
  Serial.println("Action: applyCfg2_RmMtFillOutputs");
  setOutputPin(PIN_FAN_1, HIGH);
  setOutputPin(PIN_FAN_2, HIGH);
  setOutputPin(PIN_RM_V_1, HIGH);
  setRelay(REL_V_2_3, true);
}

// -----------------------------------------------------------------------------
// Step engine helpers
// -----------------------------------------------------------------------------

bool stepTimeElapsed() {
  return millis() - stepStartMs >= stepDurationMs;
}

void startStep(Step step, unsigned long durationMs) {
  currentStep    = step;
  stepDurationMs = durationMs;
  stepStartMs    = millis();
}

// -----------------------------------------------------------------------------
// Measurement cycle
// -----------------------------------------------------------------------------

void startMeasurementCycle() {
  setAllOutputsOff();
  applyMeasurementOutputs();
  startStep(Step::Meas, MEASUREMENT_TIME_MS);
  currentMode = RoomMode::Measuring;

  // Reflect state into Modbus holding registers
  mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
  mb_write(HR_STEP,      (uint16_t)currentStep);
}

void handleMeasurement() {
  // Publish live readings each second during measurement
  static unsigned long lastPublishMs = 0;
  unsigned long now = millis();
  if (now - lastPublishMs >= 1000) {
    lastPublishMs = now;
    float o2  = readO2Percent();
    float co2 = readCO2Percent();
    Serial.print("[meas] pct O2="); Serial.print(o2, 2);
    Serial.print(" CO2="); Serial.print(co2, 2);
    Serial.println(" (publishing)");
    uint16_t o2Scaled  = (uint16_t)clamp_u16((int32_t)lroundf(o2 * 100.0f), 0, 65535);
    uint16_t co2Scaled = (uint16_t)clamp_u16((int32_t)lroundf(co2 * 100.0f), 0, 65535);
    mb_write(HR_O2_MEAS,  o2Scaled);
    mb_write(HR_CO2_MEAS, co2Scaled);
  }

  // End measurement when time elapses
  if (stepTimeElapsed()) {
    float o2  = readO2Percent();
    float co2 = readCO2Percent();
    Serial.print("Measurement finished. O2=%");
    Serial.print(o2, 2);
    Serial.print(" CO2=");
    Serial.println(co2, 2);

    setAllOutputsOff();
    currentMode = RoomMode::Idle;
    currentStep = Step::None;
    mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
    mb_write(HR_STEP,      (uint16_t)currentStep);
  }
}

// -----------------------------------------------------------------------------
// Calibration cycle
// -----------------------------------------------------------------------------

void applyCalibrationOutputs() {
  // Only measuring pump and calibration valve ON; everything else OFF
  setOutputPin(PIN_FAN_1, LOW);
  setOutputPin(PIN_FAN_2, LOW);
  setOutputPin(PIN_RM_V_1, LOW);
  setOutputPin(PIN_N2_RM_V_1, LOW);
  setRelay(REL_V_4_4,      false);
  setRelay(REL_V_2_3,      false);
  setRelay(REL_V_1_1,      false);
  setRelay(REL_M_V_N2_GEN, false);
  setRelay(REL_MEAS_V_A_I, false);
  setRelay(REL_N2_REC_V,   false);
  setRelay(REL_MEAS_P,     true);
  setRelay(REL_MEAS_V_CAL, true);
}

void startCalibrationCycle() {
  setAllOutputsOff();
  applyCalibrationOutputs();
  startStep(Step::Calibrate, CALIBRATION_TIME_MS);

  calTimingCycleId++;
  calTimingDurationS = (uint32_t)(CALIBRATION_TIME_MS / 1000UL);
  calTimingRemainingS = calTimingDurationS;
  calTimingLastUpdateMs = millis();
  publishCalibrationTimingBlock(true, true);

  currentMode = RoomMode::Calibrating;
  mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
  mb_write(HR_STEP,      (uint16_t)currentStep);
}

void handleCalibration() {
  unsigned long now = millis();

  // Keep remaining_s updates at <= 1 Hz while calibration is active.
  if ((uint32_t)(now - calTimingLastUpdateMs) >= 1000u) {
    calTimingLastUpdateMs = now;
    uint32_t elapsedMs = (uint32_t)(now - stepStartMs);
    uint32_t remainingMs = (elapsedMs >= (uint32_t)CALIBRATION_TIME_MS)
      ? 0u
      : ((uint32_t)CALIBRATION_TIME_MS - elapsedMs);
    uint32_t nextRemainingS = (remainingMs + 999u) / 1000u;
    if (nextRemainingS > calTimingRemainingS) nextRemainingS = calTimingRemainingS;
    calTimingRemainingS = nextRemainingS;
    publishCalibrationTimingBlock(true, true);
  }

  if (!stepTimeElapsed()) {
    return;
  }
  // End calibration when time elapses; resume Idle
  setAllOutputsOff();
  currentMode = RoomMode::Idle;
  currentStep = Step::None;
  mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
  mb_write(HR_STEP,      (uint16_t)currentStep);
  setCalibrationTimingIdle();
}

// Periodic serial output of latest O2/CO2 measurements (global scope)
static unsigned long lastPrintMs = 0;
static const unsigned long PRINT_INTERVAL_MS = 1000;

static void printLatestReadings() {
  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = now;
  uint16_t o2Scaled  = holdingRegs[HR_O2_MEAS];
  uint16_t co2Scaled = holdingRegs[HR_CO2_MEAS];
  float o2 = o2Scaled / 100.0f;
  float co2 = co2Scaled / 100.0f;
  Serial.print("O2="); Serial.print(o2, 2);
  Serial.print("%  CO2="); Serial.print(co2, 2);
  Serial.println("%");
}

// -----------------------------------------------------------------------------
// Aeration cycle
// -----------------------------------------------------------------------------

void startAerationCycle(float o2ErrorPercent) {
  // o2ErrorPercent is positive when O2 is below setpoint
  if (o2ErrorPercent <= 0.0f) {
    return;
  }

  unsigned long part2Duration = (unsigned long)((o2ErrorPercent / 0.1f) * (float)AER_PER_0_1PCT_MS);
  if (part2Duration == 0) {
    return;
  }

  pendingAerationPart2Time = part2Duration;

  setAllOutputsOff();
  applyAerationPart1Outputs();
  startStep(Step::AerationPart1, AER_PART1_TIME_MS);
  currentMode = RoomMode::Aerating;
  // Reflect state into Modbus holding registers immediately
  mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
  mb_write(HR_STEP,      (uint16_t)currentStep);
}

void handleAeration() {
  if (!stepTimeElapsed()) {
    return;
  }

  switch (currentStep) {
    case Step::AerationPart1:
      setAllOutputsOff();
      applyAerationPart2Outputs();
      startStep(Step::AerationPart2, pendingAerationPart2Time);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::AerationPart2:
      setAllOutputsOff();
      currentMode = RoomMode::Idle;
      currentStep = Step::None;
      mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
      mb_write(HR_STEP,      (uint16_t)currentStep);
      break;

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Scrubbing Configuration 1
// -----------------------------------------------------------------------------

void startScrubCfg1() {
  setAllOutputsOff();
  applyCfg1_AdsReg1Outputs();
  startStep(Step::Cfg1_AdsReg1, CFG1_ADSREG1_MS);
  currentMode = RoomMode::ScrubbingCfg1;
  // Reflect state into Modbus holding registers immediately
  mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
  mb_write(HR_STEP,      (uint16_t)currentStep);
}

void handleScrubCfg1() {
  if (!stepTimeElapsed()) {
    return;
  }

  switch (currentStep) {
    case Step::Cfg1_AdsReg1:
      setAllOutputsOff();
      applyCfg1_AdsReg2Outputs();
      startStep(Step::Cfg1_AdsReg2, CFG1_ADSREG2_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg1_AdsReg2:
      setAllOutputsOff();
      applyCfg1_XtrRegenOutputs();
      startStep(Step::Cfg1_XtrRegen, CFG1_XTRREGEN_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg1_XtrRegen:
      setAllOutputsOff();
      applyCfg1_WaitOutputs();
      startStep(Step::Cfg1_Wait, CFG1_WAIT_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg1_Wait:
      setAllOutputsOff();
      applyCfg1_RmMtFillOutputs();
      startStep(Step::Cfg1_RmMtFill, CFG1_RMMTFILL_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg1_RmMtFill:
      setAllOutputsOff();
      currentMode = RoomMode::Idle;
      currentStep = Step::None;
      mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
      mb_write(HR_STEP,      (uint16_t)currentStep);
      break;

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Scrubbing Configuration 2
// -----------------------------------------------------------------------------

void startScrubCfg2() {
  setAllOutputsOff();
  applyCfg2_AdsReg1Outputs();
  startStep(Step::Cfg2_AdsReg1, CFG2_ADSREG1_MS);
  currentMode = RoomMode::ScrubbingCfg2;
  // Reflect state into Modbus holding registers immediately
  mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
  mb_write(HR_STEP,      (uint16_t)currentStep);
}

void handleScrubCfg2() {
  if (!stepTimeElapsed()) {
    return;
  }

  switch (currentStep) {
    case Step::Cfg2_AdsReg1:
      setAllOutputsOff();
      applyCfg2_AdsReg2Outputs();
      startStep(Step::Cfg2_AdsReg2, CFG2_ADSREG2_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg2_AdsReg2:
      setAllOutputsOff();
      applyCfg2_XtrRegenOutputs();
      startStep(Step::Cfg2_XtrRegen, CFG2_XTRREGEN_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg2_XtrRegen:
      setAllOutputsOff();
      applyCfg2_WaitOutputs();
      startStep(Step::Cfg2_Wait, CFG2_WAIT_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg2_Wait:
      setAllOutputsOff();
      applyCfg2_RmMtFillOutputs();
      startStep(Step::Cfg2_RmMtFill, CFG2_RMMTFILL_MS);
      mb_write(HR_STEP, (uint16_t)currentStep);
      break;

    case Step::Cfg2_RmMtFill:
      setAllOutputsOff();
      currentMode = RoomMode::Idle;
      currentStep = Step::None;
      mb_write(HR_ROOM_MODE, (uint16_t)currentMode);
      mb_write(HR_STEP,      (uint16_t)currentStep);
      break;

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Arduino setup/loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  initPins();

  // USER button removed; use HMI Modbus coil to start measurement

  // Initialize Opta controller and discover expansions
  #ifdef HAVE_OPTA_BLUE
  OptaController.begin();

  // Find analog (A0602) and digital (D1608E) expansions —
  // mirror the official examples by probing typed expansions across all slots.
  for (int i = 0; i < OPTA_CONTROLLER_MAX_EXPANSION_NUM; i++) {
    if (analogExpIndex < 0) {
      AnalogExpansion ae = OptaController.getExpansion(i);
      if (ae) {
        if (ae.getType() == EXPANSION_OPTA_ANALOG) {
          analogExpIndex = i;
          Serial.print("Found Analog expansion at index "); Serial.print(i);
          Serial.print(" I2C="); Serial.println(ae.getI2CAddress());

          // Configure channels as CURRENT ADC for 4-20 mA sensors (I1=O2, I2=CO2)
          // Retry a few times; some setups need extra update cycles before the mode sticks.
          for (int attempt = 1; attempt <= 5; attempt++) {
            AnalogExpansion::beginChannelAsCurrentAdc(OptaController, analogExpIndex, 0);
            AnalogExpansion::beginChannelAsCurrentAdc(OptaController, analogExpIndex, 1);
            OptaController.update();
            delay(50);
            OptaController.update();

            AnalogExpansion ae2 = OptaController.getExpansion(analogExpIndex);
            bool ok0 = false;
            bool ok1 = false;
            if (ae2) {
              ok0 = ae2.isChCurrentAdc(0, false);
              ok1 = ae2.isChCurrentAdc(1, false);
            }
            Serial.print("A0602 current ADC attempt ");
            Serial.print(attempt);
            Serial.print(": ch0=");
            Serial.print(ok0 ? "YES" : "NO");
            Serial.print(" ch1=");
            Serial.println(ok1 ? "YES" : "NO");

            if (ok0 && ok1) {
              break;
            }
          }

          // One-time readout after configuration attempts
          {
            AnalogExpansion ae2 = OptaController.getExpansion(analogExpIndex);
            if (ae2) {
              ae2.updateAnalogInputs();
              float mA0 = ae2.pinCurrent(0, false);
              float mA1 = ae2.pinCurrent(1, false);
              Serial.print("A0602 pinCurrent mA: ch0=");
              Serial.print(mA0, 3);
              Serial.print(" ch1=");
              Serial.println(mA1, 3);
            } else {
              Serial.println("[WARN] A0602: failed to re-fetch expansion after config");
            }
          }
        }
      }
    }
    if (digitalExpIndex < 0) {
      DigitalMechExpansion mech = OptaController.getExpansion(i);
      DigitalStSolidExpansion sts = OptaController.getExpansion(i);
      if (mech) {
        digitalExpIndex = i;
        Serial.print("Found Digital expansion at index "); Serial.print(i);
        Serial.print(" type=MEC I2C="); Serial.println(mech.getI2CAddress());
      } else if (sts) {
        digitalExpIndex = i;
        Serial.print("Found Digital expansion at index "); Serial.print(i);
        Serial.print(" type=STS I2C="); Serial.println(sts.getI2CAddress());
      }
    }
  }
  if (LOG_DISCOVERY) {
    Serial.print("Detected expansions: count="); Serial.println(OptaController.getExpansionNum());
    Serial.print("A0602 index="); Serial.println(analogExpIndex);
    Serial.print("D1608E index="); Serial.println(digitalExpIndex);
  }
  #endif
  #ifndef HAVE_OPTA_BLUE
  Serial.println("[WARN] Opta expansions library not found; using stubs. A0602/D1608E inactive.");
  #endif

  // Ensure everything starts off
  setAllOutputsOff();

  // Do not auto-start a cycle at power-up; use USER button or Modbus command

  // -------------------------------------------------------------------------
  // Ethernet + Modbus TCP initialization
  // -------------------------------------------------------------------------
  Ethernet.begin(optaMac, optaIp, optaDns, optaGateway, optaSubnet);
  if (LOG_DISCOVERY) {
    Serial.print("Opta Ethernet IP: ");
    Serial.println(Ethernet.localIP());
    Serial.print("Opta Ethernet HW: ");
    EthernetHardwareStatus hw = Ethernet.hardwareStatus();
    Serial.print((int)hw);
    Serial.print(" | Link=");
    EthernetLinkStatus link = Ethernet.linkStatus();
    Serial.println(link == LinkON ? "LinkON" : (link == LinkOFF ? "LinkOFF" : "Unknown"));
  }

  // Start a tiny debug TCP server on port 5050 to validate TCP reachability
  dbgServer.begin();
  if (LOG_DISCOVERY) Serial.println("DBG TCP Server listening at 192.168.137.199:5050");

  // Temporary: start a raw TCP reachability server on 5503 to validate arbitrary port connectivity
  reachServer.begin();
  if (LOG_DISCOVERY) Serial.println("REACH TCP Server listening at 192.168.137.199:5503");

  // Note: Only ArduinoModbus should listen on 502; no additional server bound to that port.

  // Start custom Modbus TCP server on 502
  delay(2000);

  // Start Modbus TCP socket
  mbServer.begin();

#if USE_ARDUINO_MODBUS
  // Start ArduinoModbus server (no unit-id enforcement).
  if (!modbusTCPServer.begin()) {
    Serial.println("[FATAL] Failed to start ModbusTCPServer");
    while (1) { delay(10); }
  }

  // Minimal, safe register map:
  // - Coils 0..4: momentary commands from HMI/Pi
  // - Holding regs 0..199: allows reads of HR10..HR20 and HR120.. with no crashes
  modbusTCPServer.configureCoils(0, 5);
  modbusTCPServer.configureHoldingRegisters(0, HOLDING_REGS_SIZE);

  if (Serial) {
    Serial.print("[MB] Holding registers configured: HR0..");
    Serial.print((int)(HOLDING_REGS_SIZE - 1));
    Serial.println(" active (0-based addresses)");
  }

  // Initialize coils low
  for (int i = 0; i < 5; i++) {
    (void)modbusTCPServer.coilWrite(i, 0);
  }

  // Firmware identity (RO): lets clients confirm the correct firmware is running.
  {
    const uint32_t fwBuildId = 250122u;
    mb_write(HR_FW_BUILD_ID_LO, (uint16_t)(fwBuildId & 0xFFFFu));
    mb_write(HR_FW_BUILD_ID_HI, (uint16_t)((fwBuildId >> 16) & 0xFFFFu));
    // Semantic version (RO): UI-friendly major.minor.patch
    mb_write(HR_FW_VERSION_MAJOR, 1);
    mb_write(HR_FW_VERSION_MINOR, 3);
    mb_write(HR_FW_VERSION_PATCH, 0);
  }

  // Publish default calibration parameters into holding registers.
  // (Pi can read/overwrite HR30..HR35 and HR40..HR45 and then write HR50=1 to apply.)
  mb_restore_calibration_defaults();

  // Clear status/command and attempt to load saved calibration.
  // If a valid saved record exists, it overwrites HR30..HR35/HR40..HR45 and applies to runtime.
  mb_write(HR_CAL_CMD, 0);
  mb_write(HR_CAL_STATUS, 0);
  mb_write(HR_SWEEP_RELAY_ON_MS, SWEEP_RELAY_ON_MS_DEFAULT);
  setCalibrationTimingIdle();
  {
    uint16_t loadErr = 0;
    bool loaded = cal_store_load(&loadErr);
    (void)loaded;
    if (loadErr >= 100) {
      // Report load error (defaults remain active).
      mb_write(HR_CAL_STATUS, loadErr);
    }
  }

  if (LOG_DISCOVERY) Serial.println("ArduinoModbus Modbus TCP Server listening at 192.168.137.199:502");

  // Control ownership defaults.
  // Safer default for field deployments: local HMI owns control at boot.
  controlOwner = CONTROL_OWNER_HMI;
  controlCmdId = 0;
  controlLastChangeReason = CONTROL_REASON_BOOT;
  mb_write(HR_CONTROL_OWNER, controlOwner);
  mb_write(HR_CONTROL_CMD_ID, controlCmdId);
  mb_write(HR_CONTROL_LAST_CHANGE_REASON, controlLastChangeReason);
  mb_write(HR_CONTROL_LAST_CHANGE_UNIX_S_LO, 0);
  mb_write(HR_CONTROL_LAST_CHANGE_UNIX_S_HI, 0);
  mb_write(HR_HMI_HEARTBEAT, 0);
  hmiHeartbeatLastValue = 0;
  hmiHeartbeatLastChangeMs = millis();
#else
  // Legacy custom Modbus TCP server
  // (kept for fallback; not recommended)
  if (LOG_DISCOVERY) Serial.println("Custom Modbus TCP Server listening at 192.168.137.199:502 (functions 0x03,0x01,0x05)");
#endif
}

void loop() {
  // --- DIAG: ultra-light liveness (1 byte/sec) ---
  // Purpose: distinguish Opta reset/stall vs. Modbus socket abort.
  // Cost: 1 Serial byte/sec when Serial is open.
  static uint32_t diagLastHbMs = 0;
  uint32_t diagNowMs = millis();
  if ((uint32_t)(diagNowMs - diagLastHbMs) >= 1000u) {
    diagLastHbMs = diagNowMs;
    if (Serial) Serial.write('.');
  }

  // Keep Opta bus updated
  #ifdef HAVE_OPTA_BLUE
  OptaController.update();
  #endif

  // Auto-rescan expansions if they were not detected at boot (e.g., powered late)
  tryRescanExpansions();

  // Keep snapshot holding registers stable by publishing before Modbus poll().
  // This avoids writes in the post-poll response handling window.
  publishSnapshotRegistersSafe(diagNowMs);

#if USE_ARDUINO_MODBUS
  // ArduinoModbus Modbus TCP handling (single persistent client)
  // Pattern:
  // - Accept exactly one client at a time.
  // - While connected: poll every loop() iteration (do NOT gate on available()).
  // - On disconnect: stop and allow a new client.
  //
  // Verification steps (after flashing):
  // 1) QModMaster FC03 read HR10 count 2 at 1000ms -> 0 errors
  // 2) QModMaster FC03 read HR30..35 count 6 continuously -> 0 errors
  // 3) Attempt illegal write to RO (e.g., HR10): it may "stick" briefly but must revert without breaking reads
  // 4) Confirm no more "connection reset by peer" in pymodbus
  {
    static EthernetClient mbClient;
    static uint32_t mbAccepts = 0;
    static uint32_t mbDisconnects = 0;
    static uint32_t mbHealthLastPrintMs = 0;
    static bool mbWasConnected = false;

    // Edge-detect disconnect so we don't spam stop()/disconnect counters.
    bool connectedNow = mbClient.connected();
    if (mbWasConnected && !connectedNow) {
      mbDisconnects++;
      if (Serial) Serial.print("\n[MB] client disconnected");
      mbClient.stop();
      // Some Ethernet stacks can report connected() true for a while after stop()/peer loss.
      // Reset the client object so clientConnected drops immediately and a new client can be accepted.
      mbClient = EthernetClient();

      // If any deferred control-reg sync is pending, apply now (no in-flight response).
      mb_apply_control_sync_now();
    }

    // If no active client, accept one.
    if (!connectedNow) {
      if (mbClient) mbClient.stop();
      EthernetClient incoming = mbServer.available();
      if (incoming) {
        mbClient = incoming;
        mbClient.setTimeout(200);
        // IMPORTANT: accept() must be called only once per connection.
        modbusTCPServer.accept(mbClient);
        // Option A: initialize idle timer immediately on accept so idle timeout works
        // even if the client never successfully sends a Modbus request.
        mbLastReqMs = diagNowMs;
        mbAccepts++;
        if (Serial) {
          Serial.print("\n[MB] client connected from ");
          Serial.print(mbClient.remoteIP());
        }
        connectedNow = mbClient.connected();
      }
    }

    // Service the connected client.
    if (mbClient.connected()) {
      // HARD RULE: never skip poll(). We always poll first each loop iteration.
      mbPollCount++;
      int handled = modbusTCPServer.poll();
      if (handled > 0) {
        mbReqCount += (uint32_t)handled;
        mbLastReqMs = diagNowMs;
      }

      // Idle/inactivity timeout: drop client if it holds the socket but stops sending requests.
      // Must be checked AFTER poll() and AFTER updating mbLastReqMs on handled requests.
      uint32_t age = mbLastReqMs ? (uint32_t)(diagNowMs - mbLastReqMs) : 0;
      if (age > MB_IDLE_TIMEOUT_MS) {
        if (Serial) {
          Serial.print("\n[MB] idle timeout drop ageMs=");
          Serial.print(age);
        }
        mbDisconnects++;
        mbClient.stop();
        // Force release even if connected() stays true (e.g., cable unplug / peer vanished).
        mbClient = EthernetClient();
        // Apply any deferred control-reg sync now that no response is in flight.
        mb_apply_control_sync_now();
      } else {

      // Detect external writes to control ownership regs (after poll).
      // Do NOT write any registers in this same iteration; only arm a deferred sync.
      {
        bool ownerWritten = false;
        bool cmdIdWritten = false;
        bool hbWritten = false;
        uint16_t rawOwner = holdingRegs[HR_CONTROL_OWNER];
        uint16_t rawCmdId = holdingRegs[HR_CONTROL_CMD_ID];
        uint16_t rawHb = holdingRegs[HR_HMI_HEARTBEAT];

        long ownerReg = modbusTCPServer.holdingRegisterRead((int)HR_CONTROL_OWNER);
        if (ownerReg >= 0) {
          uint16_t v = (uint16_t)ownerReg;
          if (v != holdingRegs[HR_CONTROL_OWNER]) {
            holdingRegs[HR_CONTROL_OWNER] = v; // acknowledge raw write to avoid re-trigger
            rawOwner = v;
            ownerWritten = true;
          }
        }

        long cmdIdReg = modbusTCPServer.holdingRegisterRead((int)HR_CONTROL_CMD_ID);
        if (cmdIdReg >= 0) {
          uint16_t v = (uint16_t)cmdIdReg;
          if (v != holdingRegs[HR_CONTROL_CMD_ID]) {
            holdingRegs[HR_CONTROL_CMD_ID] = v; // acknowledge raw write to avoid re-trigger
            rawCmdId = v;
            cmdIdWritten = true;
          }
        }

        long hbReg = modbusTCPServer.holdingRegisterRead((int)HR_HMI_HEARTBEAT);
        if (hbReg >= 0) {
          uint16_t v = (uint16_t)hbReg;
          if (v != holdingRegs[HR_HMI_HEARTBEAT]) {
            holdingRegs[HR_HMI_HEARTBEAT] = v; // acknowledge raw write to avoid re-trigger
            rawHb = v;
            hbWritten = true;
          }
        }

        if (hbWritten) {
          // Heartbeat proof-of-life: only changes matter (wraparound OK).
          if (rawHb != hmiHeartbeatLastValue) {
            hmiHeartbeatLastValue = rawHb;
            hmiHeartbeatLastChangeMs = diagNowMs;
            // Local-first reclaim: if HMI comes alive, take ownership.
            if (controlOwner != CONTROL_OWNER_HMI) {
              setControlOwner(CONTROL_OWNER_HMI, CONTROL_REASON_USER_TAKEOVER, controlCmdId);
            }
          }
        }

        bool leaseValid = ((uint32_t)(diagNowMs - hmiHeartbeatLastChangeMs) < HMI_LEASE_TIMEOUT_MS);

        if (ownerWritten || cmdIdWritten) {
          uint16_t clamped = clampControlOwner(rawOwner);

          // Takeover guard: while HMI lease is valid, ignore attempts to set owner away from HMI.
          if (controlOwner == CONTROL_OWNER_HMI && leaseValid && clamped != CONTROL_OWNER_HMI) {
            holdingRegs[HR_CONTROL_OWNER] = controlOwner;
            holdingRegs[HR_CONTROL_CMD_ID] = controlCmdId;
            mb_arm_control_sync(controlOwner, controlCmdId, controlLastChangeReason);
          } else {
            // If an invalid owner value was written, we clamp to NONE and treat as FORCED.
            uint16_t reason = (clamped != rawOwner) ? CONTROL_REASON_FORCED : CONTROL_REASON_USER_TAKEOVER;
            setControlOwner(clamped, reason, rawCmdId);
          }
        }
      }

      // Two-phase RO enforcement (no poll skipping):
      // 1) Detect phase (after poll): scan only RO ranges and set pending flag; do NOT write.
      bool roMismatch = mb_detect_ro_violations();
      if (roMismatch && !mbRoRestorePending) {
        mbRoRestorePending = true;
        mbRoRestoreArmedAtPollCount = mbPollCount;
      }

      // 2) Restore phase: restore AFTER poll, but only once we've completed at least one
      // additional poll() since we armed the restore.
      if (mbRoRestorePending && (mbPollCount > mbRoRestoreArmedAtPollCount)) {
        mb_restore_ro_holding_registers();
        mbRoRestorePending = false;
      }

      // Apply any deferred control-reg writes once we're safely past the poll that armed them.
      mb_apply_control_sync_if_armed();
      }
    }

    mbWasConnected = mbClient.connected();

    // HMI heartbeat lease timeout: if no heartbeat changes arrive, drop HMI ownership to NONE.
    {
      uint32_t hbAgeMs = (uint32_t)(diagNowMs - hmiHeartbeatLastChangeMs);
      bool leaseValid = (hbAgeMs < HMI_LEASE_TIMEOUT_MS);
      if (controlOwner == CONTROL_OWNER_HMI && !leaseValid) {
        setControlOwner(CONTROL_OWNER_NONE, CONTROL_REASON_TIMEOUT, controlCmdId);
        if (!mbClient.connected()) {
          // No in-flight Modbus response; safe to apply immediately.
          mb_apply_control_sync_now();
        }
        if (Serial) {
          Serial.print("\n[control] TIMEOUT: owner 1->0 (no hb for ");
          Serial.print(hbAgeMs);
          Serial.println("ms)");
        }
      }
    }

    // Sweep dwell parameter clamp (RW register HR52).
    {
      long rawDwell = modbusTCPServer.holdingRegisterRead((int)HR_SWEEP_RELAY_ON_MS);
      if (rawDwell >= 0) {
        uint16_t raw = (uint16_t)rawDwell;
        uint16_t clamped = clampSweepRelayOnDurationMs((uint32_t)raw);
        if (raw != clamped || holdingRegs[HR_SWEEP_RELAY_ON_MS] != clamped) {
          mb_write(HR_SWEEP_RELAY_ON_MS, clamped);
        }
      }
    }

    // Momentary coil commands: detect set bits, act once, then reset coil.
    // (Coil/register table is shared across all clients.)
    int coil0 = modbusTCPServer.coilRead(COIL_CMD_MEASURE_START);
    int coil1 = modbusTCPServer.coilRead(COIL_CMD_CALIBRATE_START);
    int coil2 = modbusTCPServer.coilRead(COIL_CMD_SCRUB_CFG1_START);
    int coil3 = modbusTCPServer.coilRead(COIL_CMD_SCRUB_CFG2_START);
    int coil4 = modbusTCPServer.coilRead(COIL_CMD_RELAY_TEST);

    if (coil0) {
      (void)modbusTCPServer.coilWrite(COIL_CMD_MEASURE_START, 0);
      if (LOG_ACTIONS) Serial.println("[MB] coil0=1 -> startMeasurementCycle");
      if (currentMode == RoomMode::Idle) startMeasurementCycle();
    }
    if (coil1) {
      (void)modbusTCPServer.coilWrite(COIL_CMD_CALIBRATE_START, 0);
      if (LOG_ACTIONS) Serial.println("[MB] coil1=1 -> startCalibrationCycle");
      if (currentMode == RoomMode::Idle) startCalibrationCycle();
    }
    if (coil2) {
      (void)modbusTCPServer.coilWrite(COIL_CMD_SCRUB_CFG1_START, 0);
      if (LOG_ACTIONS) Serial.println("[MB] coil2=1 -> startScrubCfg1");
      if (currentMode == RoomMode::Idle) startScrubCfg1();
    }
    if (coil3) {
      (void)modbusTCPServer.coilWrite(COIL_CMD_SCRUB_CFG2_START, 0);
      if (LOG_ACTIONS) Serial.println("[MB] coil3=1 -> startScrubCfg2");
      if (currentMode == RoomMode::Idle) startScrubCfg2();
    }
    // coil4 is a level: 1=run sweep, 0=stop sweep
    static int lastCoil4 = 0;
    if (coil4 != lastCoil4) {
      lastCoil4 = coil4;
      if (coil4) {
        if (LOG_SWEEP_EVENTS) Serial.println("[MB] coil4=1 -> relay sequencer BEGIN");
        relaySweepBegin();
      } else {
        if (LOG_SWEEP_EVENTS) Serial.println("[MB] coil4=0 -> relay sequencer STOP");
        relaySweepStop();
      }
    }

    // Periodic Modbus health summary (low impact)
    if (Serial && (uint32_t)(diagNowMs - mbHealthLastPrintMs) >= 3000u) {
      mbHealthLastPrintMs = diagNowMs;
      Serial.print("\n[MB] health ms=");
      Serial.print(diagNowMs);
      Serial.print(" accepts=");
      Serial.print(mbAccepts);
      Serial.print(" disconnects=");
      Serial.print(mbDisconnects);
      Serial.print(" pollCount=");
      Serial.print(mbPollCount);
      Serial.print(" reqCount=");
      Serial.print(mbReqCount);
      Serial.print(" lastReqAgeMs=");
      Serial.print(mbLastReqMs ? (diagNowMs - mbLastReqMs) : 0);
      Serial.print(" roRestorePending=");
      Serial.print(mbRoRestorePending ? "1" : "0");
      Serial.print(" clientConnected=");
      Serial.print(mbClient.connected() ? "1" : "0");
      Serial.print(" lastReqMs=");
      Serial.println(mbLastReqMs);
    }
  }
#else
  // Legacy custom Modbus TCP handler removed from mainline (fallback only)
#endif

  // Apply calibration commands written via Modbus holding registers (HR50).
  mb_service_calibration_command();

  // Live HMI values (HR10/HR11) update at 2 Hz.
  updateSensors2Hz(diagNowMs);

  // A0602 diagnostics are mirrored into HR60..HR67 at 1 Hz.
  updateA0602Diagnostics1Hz(diagNowMs);

  // Heartbeat: print a periodic status to confirm server is running
  static unsigned long lastHeartbeat = 0;
  static unsigned long mbPolls = 0;
  mbPolls++;
  unsigned long now = diagNowMs;
  if (now - lastHeartbeat >= 5000) {
    lastHeartbeat = now;
    Serial.print("[MB] heartbeat; polls=");
    Serial.print(mbPolls);
    Serial.print("; link=");
    EthernetLinkStatus link = Ethernet.linkStatus();
    Serial.println(link == LinkON ? "LinkON" : (link == LinkOFF ? "LinkOFF" : "Unknown"));
  }

  // Reachability server: accept and close to prove port openness
  {
    EthernetClient c = reachServer.available();
    if (c) {
      Serial.println("[REACH] client connected on 5503 -> closing");
      c.stop();
    }
  }

  // No raw TCP server on 502; keep port reserved for ArduinoModbus only

  // ArduinoModbus coil handling disabled; custom server handles 0x05 writes

  // Accept and respond to debug TCP connections (port 5050)
  EthernetClient dbg = dbgServer.available();
  if (dbg) {
    Serial.println("DBG: client connected on 5050");
    dbg.println("hello from Opta");
    // give client a moment to read, then close
    delay(5);
    dbg.stop();
  }

  // Raw Modbus TCP handler disabled
  // EthernetClient c = rawMbServer.available();
  // ...

  // USER button logic removed; measurement starts via Modbus coil 0 from HMI

  switch (currentMode) {
    case RoomMode::Measuring:
      handleMeasurement();
      break;

    case RoomMode::Aerating:
      handleAeration();
      break;

    case RoomMode::ScrubbingCfg1:
      handleScrubCfg1();
      break;

    case RoomMode::ScrubbingCfg2:
      handleScrubCfg2();
      break;

    case RoomMode::Calibrating:
      handleCalibration();
      break;

    case RoomMode::Idle:
    default:
      // Later: schedule cycles or respond to HMI commands here
      break;
  }

  // Drive relay sweep state machine when active
  relaySweepTick(millis());

  // Optional: print latest readings/heartbeat
  #if 1
  if (LOG_HEARTBEAT) printLatestReadings();
  #endif
}

// -----------------------------------------------------------------------------
// Arduino IDE upload notes:
// - Tools  Board  Arduino Opta
// - Tools  Port   Opta COM port
// - Verify  Upload
// -----------------------------------------------------------------------------
