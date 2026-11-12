# Lean Drivetrain Plan

## Objective

A small, clear, drivetrain-only Reefscape codebase with:

- mk4n REV swerve today, mk5n CTRE later
- input shaping
- heading hold while strafing
- field-oriented and robot-oriented modes

Everything else is removed.

## Success criteria

- Teleop drive, field-oriented mode, slow mode, gyro reset, and pose logging work.
- Input shaping and heading hold are implemented and unit tested.
- Robot-oriented mode is a runtime toggle.
- `Swerve` depends on `SwerveModuleIO`, not REV/CTRE APIs.
- No elevator, algae, coral, climber, lights, PathPlanner, or PhotonVision code.
- No tracked logs or build output.
- `./gradlew build` passes; `simulateJava` smoke test passes.
- File/line count well below the 40-file / 3,633-line baseline.

## Phase 0 — Baseline and removal

1. Record the baseline commit and build result.
2. Delete tracked `logs/*.wpilog` (79 files, ~485 MB).
3. Delete `simgui-ds.json`, `networktables.json`, and other local-only files.
4. Delete mechanism packages, commands, constants, Lights, AutoChooser,
   PathPlanner deploy assets, and the related vendordeps.
5. Delete PhotonVision and all vision/field/reef/station code.
6. Keep AdvantageKit, Phoenix6 (Pigeon 2), REVLib, and WPILibNewCommands.

## Phase 1 — Drivetrain core

1. Introduce `SwerveModuleIO`.
2. Make `SwerveModule` the REV implementation.
3. Keep `Swerve` on kinematics + IO; remove vendor details.
4. Simplify `Odometry` to Pigeon 2 + `SwerveDrivePoseEstimator`.
5. Remove unused methods: PathPlanner chassis speeds, reef edges, station edges,
   wheel-radius characterization, dead buttons.
6. Add a `SwerveModuleIO` fake for tests if useful.

## Phase 2 — Features

1. Input shaping:
   - deadband
   - expo/cubic curve
   - slew limit
   - clamp
2. Heading hold:
   - PID on heading while translating
   - release on rotation input
   - reset on teleop init
3. Robot-oriented toggle:
   - runtime boolean in `SwerveTeleOp`
   - binding in `RobotContainer`
   - default field-oriented

## Phase 3 — Verification

1. Unit tests for `InputShaper` and `HeadingHold`.
2. `./gradlew build`.
3. `simulateJava`: forward, strafe with heading hold, rotate, toggle modes.
4. Feature checklist:
   - drive
   - slow mode
   - gyro reset
   - field-oriented
   - robot-oriented
   - heading hold
   - pose logging
5. Final diff audit: no dead code, no unused imports, no vendor leftovers.

## Open questions

- Is the 2025 robot available for deploy/testing, or is simulation the only check?
- Add `CTRESwerveModule` now, or only the interface and a stub?
- Keep AdvantageKit replay, or only real/sim logging?
- Fresh repo history or preserve upstream history? Recommend fresh history so the
  485 MB log blob never enters the target repo.

## Guardrails

- No CAN ID or inversion changes without approval.
- No deploy without build + tests + sim + approval.
- No new dependency for something WPILib already provides.