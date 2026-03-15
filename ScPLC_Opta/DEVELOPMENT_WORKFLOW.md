# Development Workflow

This document defines the required development workflow for the ScPLC Opta firmware and Pi server projects.

## 1. Local-First Rule

- All source code changes are made only in local desktop repositories.
- Firmware changes are made in `ScPLC_Opta`.
- Pi server changes are made in `Pi_Server`.
- No direct server source code edits are made on the Pi host.

## 2. Opta Firmware Flow

- Make code changes locally in `ScPLC_Opta`.
- Flash and test on Opta hardware from desktop using Arduino CLI.
- Agent flashing rule (mandatory): when Opta flashing is needed, the agent always uses `arduino-cli` from the desktop workflow (never IDE/manual upload flows).
- Required CLI sequence for flashing tasks:
  - Detect board/port (`arduino-cli board list`).
  - Compile with the correct FQBN (`arduino:mbed_opta:opta`).
  - Upload to the detected serial port with `arduino-cli upload`.
- If tests pass:
  - Commit locally.
  - Push branch to GitHub.
  - Open a pull request.
  - Merge after review and required checks pass.

## 3. Pi Server Flow

- Create a feature branch locally in `Pi_Server`.
- Make code changes locally only.
- Commit locally.
- Push branch to GitHub.
- Open a pull request.
- Use the SSH agent to pull that branch onto the local test Pi and run runtime validation on hardware.
- If branch tests pass on the local test Pi:
  - Merge from desktop workflow (local and GitHub).
  - Use the SSH agent to pull `main` on the field/customer Pi.
  - Run final validation on field/customer Pi against `main`.

## 4. Pi Role Model (Test vs Field)

- Treat every Pi as if it were customer-field infrastructure, including local development hardware.
- Local test Pi is used for pre-merge branch validation only.
- Field/customer Pi is updated from `main` only after PR merge.
- Promotion path is mandatory:
  - Desktop feature branch -> local test Pi branch test -> PR merge -> `main` -> field/customer Pi pull of `main`.
- No direct source code edits are made on any Pi (test or field).

## 5. SSH Agent Policy

- The SSH agent is used for Pi commands, prompts, deployment pulls, and operational checks.
- The SSH agent is not used to modify server source code.
- The Pi remote repository is configured to reject pushes from the Pi.

## 6. Merge Gate

- No merge until target-environment validation passes.
- Opta merge gate: hardware flash test passes.
- Pi merge gate: branch test on Pi passes, then post-merge `main` test passes.

## 7. Hygiene (High Priority, Mandatory)

- Hygiene is enforced at every stage and is a merge blocker.
- Keep branches focused and small; avoid unrelated changes in a pull request.
- Use clear commit messages and keep commits logically grouped.
- Run required lint, formatting, and tests before pull request and before merge.
- Do not bypass review, validation, or deployment checks.
- Keep repository documentation aligned with actual operational behavior.
- If hygiene standards are not met, fix hygiene first, then continue feature work.

## 8. Branch/PR Iteration Pattern (Current Practice)

- For each requested feature or change set, create a new feature branch from `main`.
- Keep all follow-up fixes for that feature on the same branch/PR until the user confirms it passes.
- Push incremental commits as feedback comes in (do not open a new PR for each tweak on the same feature).
- Do not merge on "looks good" alone; merge only after explicit pass confirmation (for example: "all passed. merge").
- After merge:
  - Verify `origin/main` contains the merge commit.
  - Delete the feature branch (local and remote).
  - Confirm repo state is clean on `main`.
- For the next unrelated request, start a fresh feature branch and repeat the cycle.
