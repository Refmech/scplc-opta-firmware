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
- Use the SSH agent to pull that branch onto the Pi and run runtime validation on the Pi.
- If Pi branch tests pass:
  - Merge from desktop workflow (local and GitHub).
  - Use the SSH agent to pull `main` on the Pi.
  - Run final validation on Pi against `main`.

## 4. SSH Agent Policy

- The SSH agent is used for Pi commands, prompts, deployment pulls, and operational checks.
- The SSH agent is not used to modify server source code.
- The Pi remote repository is configured to reject pushes from the Pi.

## 5. Merge Gate

- No merge until target-environment validation passes.
- Opta merge gate: hardware flash test passes.
- Pi merge gate: branch test on Pi passes, then post-merge `main` test passes.

## 6. Hygiene (High Priority, Mandatory)

- Hygiene is enforced at every stage and is a merge blocker.
- Keep branches focused and small; avoid unrelated changes in a pull request.
- Use clear commit messages and keep commits logically grouped.
- Run required lint, formatting, and tests before pull request and before merge.
- Do not bypass review, validation, or deployment checks.
- Keep repository documentation aligned with actual operational behavior.
- If hygiene standards are not met, fix hygiene first, then continue feature work.
