package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Robot-wide constants. Drivetrain only. */
public final class Constants {

	public enum Mode {
		REAL, SIM
	}

	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
	public static final double kLoopPeriodSecs = 0.02;

	public static final class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
		public static final double kDriveMotorGearRatio = 1 / 5.9;
		public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);

		public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
		public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
		public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
		public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

		public static final double kPTurning = 0.25;
	}

	public static final class DriveConstants {
		public static final double kTrackWidth = Units.inchesToMeters(20.5);
		public static final double kWheelBase = Units.inchesToMeters(20.5);

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final boolean kFrontLeftDriveEncoderReversed = true;
		public static final boolean kFrontLeftTurningEncoderReversed = true;
		public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
		public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;

		public static final boolean kFrontRightDriveEncoderReversed = true;
		public static final boolean kFrontRightTurningEncoderReversed = true;
		public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
		public static final int kFrontRightDriveAbsoluteEncoderPort = 1;

		public static final boolean kBackLeftDriveEncoderReversed = true;
		public static final boolean kBackLeftTurningEncoderReversed = true;
		public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
		public static final int kBackLeftDriveAbsoluteEncoderPort = 2;

		public static final boolean kBackRightDriveEncoderReversed = true;
		public static final boolean kBackRightTurningEncoderReversed = true;
		public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
		public static final int kBackRightDriveAbsoluteEncoderPort = 3;

		public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.854 - 0.494 - 0.548 - 2.987 - 0.02;
		public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -2.231 + 0.060 - 0.05 - 2.83 + Math.PI;
		public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.022 + 0.115 - 0.092 - 0.01;
		public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.349 + 0.324 + 1.029 + 0.672 - 1.961
				- 0.082 + 0.125 + 0.07;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;

		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 4.5;

		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
	}

	public static final class InputConstants {
		public static final double kTranslationDeadband = 0.1;
		public static final double kRotationDeadband = 0.1;
		public static final double kExpo = 0.35;
		public static final double kSlowModeScale = 0.25;
	}

	public static final class HeadingHoldConstants {
		public static final double kP = 4.0;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kMaxAngularSpeedRadPerSec = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
		public static final double kMinTranslationSpeedMps = 0.05;
		public static final double kRotationDeadband = InputConstants.kRotationDeadband;
	}

	public static final class CAN {
		public static final int kFrontLeftDriveMotorPort = 10;
		public static final int kFrontLeftTurningMotorPort = 20;

		public static final int kFrontRightDriveMotorPort = 11;
		public static final int kFrontRightTurningMotorPort = 21;

		public static final int kBackLeftDriveMotorPort = 12;
		public static final int kBackLeftTurningMotorPort = 22;

		public static final int kBackRightDriveMotorPort = 13;
		public static final int kBackRightTurningMotorPort = 23;

		public static final int kPigeonPort = 31;
	}
}
