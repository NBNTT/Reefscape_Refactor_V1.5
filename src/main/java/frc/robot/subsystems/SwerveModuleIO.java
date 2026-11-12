package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Hardware boundary for one swerve module. */
public interface SwerveModuleIO {
	void setDesiredState(SwerveModuleState desiredState);

	SwerveModuleState getState();

	SwerveModulePosition getPosition();

	double getAbsoluteEncoderAngle();

	void resetEncoders();

	void stop();
}
