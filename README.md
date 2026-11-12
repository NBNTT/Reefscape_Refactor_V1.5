# Simple Reefscape

Drivetrain-only lean rebuild of FRC team **8567**'s 2025 Reefscape robot.

- **Base:** `ultraviolet8567/Reefscape`, branch `learning`
- **Target:** `NBNTT/Simple_Reefscape`
- **Hardware:** mk4n REV swerve now; future mk5n CTRE
- **Stack:** Java 17, WPILib/GradleRIO 2025.3.1, AdvantageKit, Phoenix6 25.3.1, REVLib

## Scope

Keep only the drivetrain:

- Swerve modules and kinematics
- Pigeon 2 heading / pose estimation
- Teleop drive
- Input shaping
- Heading hold while strafing
- Field-oriented and robot-oriented modes

Removed because the robot no longer has them: elevator, algae intake, coral
intake, climber, lights, PathPlanner, PhotonVision, autonomous paths.

## Commands

```bash
./gradlew build          # compile + Spotless + tests
./gradlew test           # JUnit 5
./gradlew simulateJava   # desktop sim (run on the host)
./gradlew deploy         # only after build/test/sim and explicit approval
```

Spotless runs automatically before `compileJava` and normalizes Java formatting.

## Controls

- Left stick: translate
- Right stick X: rotate
- Right bumper: slow mode
- Y: toggle field-oriented / robot-oriented
- Back: reset gyro heading

Field-oriented is the default. Cardinal Lock captures the heading when you begin
strafing and holds it until you rotate or stop.

## Controller

8BitDo Ultimate Wireless in **XInput mode**, behind `DriveController`. Only the
left stick, right stick X, right bumper, Y, and Back are bound. Every other
button is intentionally inert.

## Reliability

Each swerve module checks its REV CAN/sensor faults every loop. On failure it is
set to coast, its last good position is frozen in odometry, and drive commands
skip that module. It recovers automatically after 0.5 seconds of stable
connection, re-seeding its drive position so the pose does not jump.

## Layout

```text
src/main/java/frc/robot/
  Main, Robot, RobotContainer, Constants
  commands/SwerveTeleOp.java
  controllers/DriveController.java, EightBitDoController.java
  subsystems/Swerve.java, SwerveModule.java, SwerveModuleIO.java, Odometry.java
  util/InputShaper.java, CardinalLock.java
src/test/java/frc/robot/    pure unit tests
vendordeps/                 AdvantageKit, Phoenix6, REVLib, WPILibNewCommands
```

## Remotes

```text
origin    https://github.com/NBNTT/Simple_Reefscape.git   (target)
upstream  https://github.com/ultraviolet8567/Reefscape.git (reference)
```

## Rule

Delete before add. Keep the drivetrain feature surface. Prove every behavior
change with a test or simulation.