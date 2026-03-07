# Release checklist (Opta firmware)

## Pre-merge (PR)

- Builds (CI) and compiles in Arduino IDE.
- Flashed to real hardware (Opta) using Arduino IDE.
- Verified basic comms:
  - Opta IP reachable.
  - Modbus TCP read works.
  - Firmware identity registers (HR5..HR9) match expectations.
- Verified any changed I/O behavior (including expansions if relevant).

## Merge + tag

- Merge PR to `main`.
- Create an annotated tag for the known-good firmware (pick one convention and stick to it):
  - Date based: `opta-vYYYY.MM.DD`
  - Semver: `vMAJOR.MINOR.PATCH`
- (Optional) Create a GitHub Release from the tag and paste the PR notes.

## Post-merge sanity

- Flash the merged `main` commit (or the tag) and re-check the same baseline tests.
