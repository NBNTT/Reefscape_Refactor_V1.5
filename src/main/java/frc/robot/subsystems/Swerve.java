package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/** Four swerve modules, independent of the module vendor. */
public class Swerve extends SubsystemBase {
	private final SwerveModuleIO frontLeft;
	private final SwerveModuleIO frontRight;
	private final SwerveModuleIO backLeft;
	private final SwerveModuleIO backRight;

	public Swerve() {
		frontLeft = new SwerveModule(CAN.kFrontLeftDriveMotorPort, CAN.kFrontLeftTurningMotorPort,
				DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningEncoderReversed,
				DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
				DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
				DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
		frontRight = new SwerveModule(CAN.kFrontRightDriveMotorPort, CAN.kFrontRightTurningMotorPort,
				DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningEncoderReversed,
				DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
				DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
				DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
		backLeft = new SwerveModule(CAN.kBackLeftDriveMotorPort, CAN.kBackLeftTurningMotorPort,
				DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningEncoderReversed,
				DriveConstants.kBackLeftDriveAbsoluteEncoderPort, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
				DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
		backRight = new SwerveModule(CAN.kBackRightDriveMotorPort, CAN.kBackRightTurningMotorPort,
				DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningEncoderReversed,
				DriveConstants.kBackRightDriveAbsoluteEncoderPort,
				DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
				DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Swerve/Measured", getModuleStates());
		Logger.recordOutput("Swerve/AbsoluteEncoders",
				new double[]{frontLeft.getAbsoluteEncoderAngle(), frontRight.getAbsoluteEncoderAngle(),
						backLeft.getAbsoluteEncoderAngle(), backRight.getAbsoluteEncoderAngle()});
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
				backRight.getPosition()};
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(),
				backRight.getState()};
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
	}

	public void setModuleStates(ChassisSpeeds chassisSpeeds) {
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		backLeft.setDesiredState(desiredStates[2]);
		backRight.setDesiredState(desiredStates[3]);
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
		frontLeft.resetEncoders();
		frontRight.resetEncoders();
		backLeft.resetEncoders();
		backRight.resetEncoders();
	}

	public void stopModules() {
		frontLeft.stop();
		frontRight.stop();
		backLeft.stop();
		backRight.stop();
	}
}
