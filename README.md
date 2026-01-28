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
   - `OptaBlue` (Opta expansion support)
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

