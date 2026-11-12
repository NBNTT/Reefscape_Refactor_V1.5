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

Field-oriented is the default. Heading hold engages automatically when you
translate without commanding rotation.

## Layout

```text
src/main/java/frc/robot/
  Main, Robot, RobotContainer, Constants
  commands/SwerveTeleOp.java
  subsystems/Swerve.java, SwerveModule.java, SwerveModuleIO.java, Odometry.java
  util/InputShaper.java, HeadingHold.java
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