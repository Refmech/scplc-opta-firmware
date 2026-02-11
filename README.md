# ScPLC Opta Firmware

This repository contains the Arduino firmware for the **ScPLC Opta** controller (Arduino Opta) only.

- Main sketch: [ScPLC_Opta/ScPLC_Opta.ino](ScPLC_Opta/ScPLC_Opta.ino)
- Pin mapping helpers: [ScPLC_Opta/io_pins.h](ScPLC_Opta/io_pins.h)

## Hardware

- Arduino **Opta** (Mbed OS core)
- Optional Opta expansions on the backplane:
  - **A0602** analog expansion (4–20 mA current inputs)
  - **D1608E** digital relay expansion

## Features (high-level)

- Non-blocking state machine for measurement, aeration, and scrubbing cycles
- Modbus TCP server for HMI/PLC integration
- Static Ethernet configuration (edit in the sketch)
- Sensor calibration parameters via Modbus holding registers

## Build / Upload

### Arduino IDE

1. Open [ScPLC_Opta/ScPLC_Opta.ino](ScPLC_Opta/ScPLC_Opta.ino) in Arduino IDE.
2. Select the **Arduino Opta** board (Mbed OS / Arduino Mbed core).
3. Install required libraries via Library Manager (names as used in code):
   - `Ethernet`
   - `ArduinoRS485`
   - `ArduinoModbus`
  - `Arduino_Opta_Blueprint` (provides `OptaBlue.h` for Opta expansions)
4. Compile and upload.

### Notes

- Calibration persistence uses `mbed::FlashIAP`, so the sketch requires an **Mbed-based** Opta core (`ARDUINO_ARCH_MBED`).

## Networking

The firmware uses a **static IP** configured in the sketch (near the top):

- IP: `192.168.137.199`
- Modbus TCP port: `502`
- Debug TCP port: `5050`
- Temporary reachability TCP port: `5503`

If you deploy onto a different subnet, update `optaIp`, `optaDns`, `optaGateway`, and `optaSubnet` in the sketch.

## Modbus TCP Interface

### Connection behavior (single client + idle timeout)

- The Opta runs a **single-client** Modbus TCP server (only one TCP client is serviced at a time).
- To prevent a “stuck connected” client (e.g., HMI sleeping while keeping its TCP socket open, or a dead peer/cable unplug), the firmware enforces an **inactivity timeout**:
  - If no Modbus requests are handled for longer than `MB_IDLE_TIMEOUT_MS`, the firmware force-drops the client.
  - Default: `MB_IDLE_TIMEOUT_MS = 5000` (5 seconds).
  - This drop is **forced even if the TCP stack still reports the socket as connected**.

Diagnostics you’ll see on Serial:
- `mbLastReqMs` is the last timestamp (ms) when a Modbus request was handled.
- `lastReqAgeMs` is the age since the last handled Modbus request (it increases when the client is idle/silent).
- When the timeout triggers, the firmware prints: `[MB] idle timeout drop ageMs=...`
- Expected behavior: when the HMI goes idle or the client disappears, `clientConnected` will drop back to `0` within ~5 seconds, allowing another client (e.g., the Pi) to connect.

### Addressing conventions

- The sketch uses **0-based holding register addresses** internally.
- Documentation / HMIs often display holding registers as **4xxxx** (1-based). For example:
  - `HR_O2_MEAS = 10` corresponds to **40011** in many HMIs.

### Coils (0-based)

- `0` (00001): Start measurement
- `1` (00002): Start calibration
- `2` (00003): Start scrub config 1
- `3` (00004): Start scrub config 2
- `4` (00005): Relay sweep test start/stop

### Key holding registers (0-based)

Read-only (firmware-owned):
- `5..6`: Firmware build id (u32 split across two u16)
- `7..9`: Semantic firmware version (major/minor/patch)
- `10`: O2 measured value (`% * 100`)
- `11`: CO2 measured value (`% * 100`)
- `19`: Room mode
- `20`: Current step
- `60..71`: Diagnostics (A0602 raw/mA/%/mode and CO2 mismatch checks)
- `180`: Outputs bitmask (`outputs_mask`)
  - bit0 D0 (PIN_FAN_1)
  - bit1 D1 (PIN_FAN_2)
  - bit2 D2 (PIN_RM_V_1)
  - bit3 D3 (PIN_N2_RM_V_1)
  - bit4 Relay1 (REL_MEAS_P)
  - bit5 Relay2 (REL_V_4_4)
  - bit6 Relay3 (REL_V_2_3)
  - bit7 Relay4 (REL_V_1_1)
  - bit8 Relay5 (REL_M_V_N2_GEN)
  - bit9 Relay6 (REL_MEAS_V_CAL)
  - bit10 Relay7 (REL_MEAS_V_A_I)
  - bit11 Relay8 (REL_N2_REC_V)

Read/write (setpoints/config):
- `30..35`: O2 calibration/config
- `40..45`: CO2 calibration/config
- `50`: Calibration command (`1=apply`, `2=save`, `3=restore defaults`)
- `51`: Calibration status/result

Control arbitration (reserved block):
- `100`: `CONTROL_OWNER` (`0=NONE`, `1=HMI`, `2=PI`)
- `101`: `CONTROL_CMD_ID`
- `105`: `HMI_HEARTBEAT` (HMI increments; used for timeout/lease)

For the authoritative definitions, see the register map comment block in [ScPLC_Opta/ScPLC_Opta.ino](ScPLC_Opta/ScPLC_Opta.ino).

## I/O Mapping

Digital inputs/outputs and relay indices are defined in:
- [ScPLC_Opta/io_pins.h](ScPLC_Opta/io_pins.h)

## Troubleshooting

- Confirm Ethernet link status on boot via Serial output.
- If expansions are connected, the firmware periodically rescans for A0602/D1608E and prints discovery logs.

