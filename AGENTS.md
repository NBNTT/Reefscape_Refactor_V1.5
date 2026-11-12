# Simple Reefscape — Agent Rules

Drivetrain-only lean rebuild of FRC team **8567**'s 2025 Reefscape robot.
Java 17, WPILib/GradleRIO **2025.3.1**, command-based.

- `origin` → `https://github.com/NBNTT/Simple_Reefscape.git` (target)
- `upstream` → `https://github.com/ultraviolet8567/Reefscape.git` (reference)
- Base branch: `learning`

## Scope

Keep only the drivetrain:

- Four swerve modules, kinematics, module states/positions
- Pigeon 2 heading and pose estimation
- Teleop drive
- Input shaping
- Heading hold while translating
- Field-oriented and robot-oriented modes

Remove: elevator, algae intake, coral intake, climber, lights, PathPlanner,
PhotonVision, autonomous paths, and their commands/constants/vendordeps.

Hardware: **mk4n REV** now, **mk5n CTRE** later. Commands must not depend on the
module vendor.

## Current state

- 14 Java files, ~891 lines.
- 11 unit tests: cubic shaping, Cardinal Lock, module failure/recovery.
- 8BitDo controller behind `DriveController`.
- Per-module CAN/sensor fault handling: coast, freeze odometry, automatic recovery.
- `SwerveModuleIO` boundary ready for a future CTRE module implementation.

## Read first

- `Constants.java` — tunables, CAN IDs, geometry.
- `RobotContainer.java` — construction and bindings.
- `Swerve`, `SwerveModule`, `Odometry`, `SwerveTeleOp` — drivetrain.
- `build.gradle` — Spotless and dependencies.

## Non-negotiables

- Preserve the drivetrain feature surface: teleop drive, field-oriented mode,
  slow mode, gyro reset, pose logging.
- Preserve CAN IDs, motor inversions, encoder offsets, and gear ratios unless
  explicitly approved.
- Keep the 2025 field convention. Do not copy 2026 geometry.
- Keep the IO boundary: `Swerve` talks to `SwerveModuleIO`, not to REV or CTRE
  APIs directly.
- Use tabs; Spotless enforces formatting.
- No blocking in periodic methods.
- No deploy without a successful `./gradlew build` and explicit approval.

## Refactor rules

- One concern per change. No drive-by rewrites.
- Delete before add. Remove unused classes, commands, constants, imports, and
  vendordeps.
- Keep the smallest complete implementation.
- New logic gets a pure unit test.
- Behavior changes need a characterization check first.
- After each batch: `./gradlew build`, then `simulateJava` when behavior changed.

## Required features

### Input shaping

- Deadband translation and rotation separately.
- Expo/cubic response, full output at full stick.
- Slew-rate limit and clamp.
- Keep the slow-mode modifier.
- Pure functions, unit tested.

### Heading hold

- While translating with no rotation command, hold the current heading with a
  PID controller.
- Release immediately when the driver commands rotation.
- Reset the target on teleop init; never snap the robot on enable.
- Tuneable deadband so stick noise does not fight the controller.

### Robot-oriented mode

- Runtime toggle between field-oriented and robot-oriented.
- Default field-oriented.
- Preserve slow mode and all existing driver bindings.
- Remove the static `Constants.fieldOriented` behavior.

## Drivetrain abstraction

- `SwerveModuleIO`: desired state, measured state, position, absolute angle,
  reset, stop.
- `SwerveModule` is the mk4n REV implementation.
- A future `CTRESwerveModule` implements the same interface; commands and
  kinematics never change.
- `Swerve` owns four `SwerveModuleIO` instances.

## Verification

- `./gradlew build` — compile + Spotless + tests.
- `./gradlew test` — pure math and behavior tests.
- `simulateJava` on the host — GUI/sim behavior.
- Only the robot proves hardware. Never claim a deploy or mechanism works
  without evidence.