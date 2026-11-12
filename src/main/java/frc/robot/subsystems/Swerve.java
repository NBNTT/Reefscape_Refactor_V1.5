package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FailureConstants;
import org.littletonrobotics.junction.Logger;

/** Four swerve modules with vendor-neutral failure compensation. */
public class Swerve extends SubsystemBase {
	private static final int MODULE_COUNT = 4;

	private final SwerveModuleIO[] modules;
	private final SwerveModulePosition[] lastGoodPositions = new SwerveModulePosition[MODULE_COUNT];
	private final SwerveModuleState[] lastGoodStates = new SwerveModuleState[MODULE_COUNT];
	private final boolean[] failed = new boolean[MODULE_COUNT];
	private final double[] healthySince = new double[MODULE_COUNT];

	public Swerve() {
		this(createModules());
	}

	/** Test seam: inject fake modules without touching hardware. */
	Swerve(SwerveModuleIO[] modules) {
		this.modules = modules;
		for (int i = 0; i < MODULE_COUNT; i++) {
			lastGoodPositions[i] = modules[i].getPosition();
			lastGoodStates[i] = modules[i].getState();
		}
	}

	private static SwerveModuleIO[] createModules() {
		return new SwerveModuleIO[]{
				new SwerveModule(CAN.kFrontLeftDriveMotorPort, CAN.kFrontLeftTurningMotorPort,
						DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningEncoderReversed,
						DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
						DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
						DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed),
				new SwerveModule(CAN.kFrontRightDriveMotorPort, CAN.kFrontRightTurningMotorPort,
						DriveConstants.kFrontRightDriveEncoderReversed,
						DriveConstants.kFrontRightTurningEncoderReversed,
						DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
						DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
						DriveConstants.kFrontRightDriveAbsoluteEncoderReversed),
				new SwerveModule(CAN.kBackLeftDriveMotorPort, CAN.kBackLeftTurningMotorPort,
						DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningEncoderReversed,
						DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
						DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
						DriveConstants.kBackLeftDriveAbsoluteEncoderReversed),
				new SwerveModule(CAN.kBackRightDriveMotorPort, CAN.kBackRightTurningMotorPort,
						DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningEncoderReversed,
						DriveConstants.kBackRightDriveAbsoluteEncoderPort,
						DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
						DriveConstants.kBackRightDriveAbsoluteEncoderReversed)};
	}

	@Override
	public void periodic() {
		for (int i = 0; i < MODULE_COUNT; i++) {
			updateModule(i);
		}

		Logger.recordOutput("Swerve/Measured", lastGoodStates);
		Logger.recordOutput("Swerve/ModuleFailed", failed);
		Logger.recordOutput("Swerve/AbsoluteEncoders",
				new double[]{modules[0].getAbsoluteEncoderAngle(), modules[1].getAbsoluteEncoderAngle(),
						modules[2].getAbsoluteEncoderAngle(), modules[3].getAbsoluteEncoderAngle()});
	}

	private void updateModule(int index) {
		SwerveModuleIO module = modules[index];
		boolean healthy = module.isHealthy();

		if (failed[index]) {
			if (!healthy) {
				healthySince[index] = 0;
				return;
			}
			// Require a stable reconnect before trusting the module again.
			if (healthySince[index] == 0) {
				healthySince[index] = nowSeconds();
			}
			if (nowSeconds() - healthySince[index] < FailureConstants.kRecoverySeconds) {
				return;
			}
			module.recover(lastGoodPositions[index]);
			failed[index] = false;
			healthySince[index] = 0;
			return;
		}

		if (!healthy) {
			failed[index] = true;
			healthySince[index] = 0;
			module.setCoast();
			module.stop();
			return;
		}

		lastGoodPositions[index] = module.getPosition();
		lastGoodStates[index] = module.getState();
	}

	private static double nowSeconds() {
		return System.nanoTime() / 1e9;
	}

	public SwerveModulePosition[] getModulePositions() {
		return lastGoodPositions;
	}

	public SwerveModuleState[] getModuleStates() {
		return lastGoodStates;
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(lastGoodStates);
	}

	public void setModuleStates(ChassisSpeeds chassisSpeeds) {
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		// A failed module must not distort desaturation for the healthy three.
		for (int i = 0; i < MODULE_COUNT; i++) {
			if (failed[i]) {
				desiredStates[i] = new SwerveModuleState();
			}
		}
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
		for (int i = 0; i < MODULE_COUNT; i++) {
			if (!failed[i]) {
				modules[i].setDesiredState(desiredStates[i]);
			}
		}
		Logger.recordOutput("Swerve/Setpoints", desiredStates);
	}

	/** Points the wheels inward so the robot resists being pushed. */
	public void lockWheels() {
		setModuleStates(new SwerveModuleState[]{new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
				new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
				new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
				new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
	}

	public void resetEncoders() {
		for (SwerveModuleIO module : modules) {
			module.resetEncoders();
		}
	}

	public void stopModules() {
		for (SwerveModuleIO module : modules) {
			module.stop();
		}
	}
}
