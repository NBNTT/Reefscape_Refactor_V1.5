package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Hardware boundary for one swerve module. Light enough to swap REV for CTRE
 * later.
 */
public interface SwerveModuleIO {
	void setDesiredState(SwerveModuleState desiredState);

	SwerveModuleState getState();

	SwerveModulePosition getPosition();

	double getAbsoluteEncoderAngle();

	/**
	 * False when the controller or its encoder reports a connection/sensor fault.
	 */
	boolean isHealthy();

	void setCoast();

	void setBrake();

	/**
	 * Re-seed the drive position from the last good odometry value after a
	 * reconnect.
	 */
	void recover(SwerveModulePosition lastGoodPosition);

	void resetEncoders();

	void stop();
}
