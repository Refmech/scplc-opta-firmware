# Relay Sweep Validation: HR19/HR20 vs HR180

## 1) Summary of observed behavior

Observed during relay sweep validation:

- `HR52=1000`
- `HR53=0` (runtime effective mask = ALL outputs)
- Coil 4 set to `1` starts relay sweep
- `HR180` changes sequentially (`1, 2, 4, 8, 16, 32, 64, ...`) as outputs are energized
- `HR19` and `HR20` remain `0/0` while sweep is active

This is expected for current firmware behavior.

## 2) Source findings

### Register mapping

- `HR19` maps to `HR_ROOM_MODE`
- `HR20` maps to `HR_STEP`
- These registers represent lifecycle process state for measurement/calibration/aeration/scrub flows.

### Sweep state machine separation

Relay sweep is implemented as a separate internal state machine (`relaySweep*` functions/state), gated by Coil 4.

Key point:

- Sweep activity **does not** update lifecycle mode/step registers (`HR19/HR20`).
- Sweep activity is observable through output snapshots (`HR180`).

So a valid sweep session can run with lifecycle still Idle/None (`HR19/HR20 = 0/0`).

## 3) Updated validation checklist

### Pre-sweep

- Confirm lifecycle idle baseline:
  - `HR19 == 0`
  - `HR20 == 0`
- Confirm sweep config:
  - `HR52` in expected dwell range
  - `HR53` set as intended (`0` means effective ALL at runtime)

### During sweep (Coil 4 = 1)

- Verify `HR180` changes over time (bitmask movement across D0..D3 and R1..R8)
- Verify `HR19/HR20` may remain unchanged at lifecycle idle values (`0/0`)
- Treat `HR180` as primary sweep activity indicator

### Stop sweep (Coil 4 = 0)

- Verify sweep stops and outputs de-energize
- Verify `HR180` settles accordingly (typically back to no active outputs)
- Lifecycle registers remain lifecycle-only indicators

## 4) Example mbpoll command snippets (read-only)

Examples below are read-only polling patterns.

- Read lifecycle mode/step (`HR19..HR20`):
  - `mbpoll -m tcp 192.168.137.199 -p 502 -t 3 -r 19 -c 2`
- Read sweep observability mask (`HR180`):
  - `mbpoll -m tcp 192.168.137.199 -p 502 -t 3 -r 180 -c 1`
- Read sweep parameters (`HR52..HR53`):
  - `mbpoll -m tcp 192.168.137.199 -p 502 -t 3 -r 52 -c 2`

If using zero-based addressing mode (`-0`), adjust register indices accordingly.

## 5) Note on contract wording

Contract wording should be interpreted as:

- `HR19/HR20` = **lifecycle mode/step only** (measurement/calibration/aeration/scrub)
- Relay sweep uses a separate state machine and intentionally does not drive `HR19/HR20`
- `HR180` is the practical runtime indicator for relay sweep activity
