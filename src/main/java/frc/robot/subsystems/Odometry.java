package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/** Pigeon 2 heading plus swerve odometry. */
public class Odometry extends SubsystemBase {
	private final Swerve swerve;
	private final Pigeon2 gyro;
	private final SwerveDrivePoseEstimator poseEstimator;

	public Odometry(Swerve swerve) {
		this.swerve = swerve;
		gyro = new Pigeon2(Constants.CAN.kPigeonPort);
		gyro.reset();
		poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getRotation2d(),
				swerve.getModulePositions(), new Pose2d());
	}

	@Override
	public void periodic() {
		poseEstimator.update(gyro.getRotation2d(), swerve.getModulePositions());

		Logger.recordOutput("Odometry/Pose", getPose());
		Logger.recordOutput("Odometry/Heading", getHeading());
		Logger.recordOutput("Odometry/GyroHeading", getGyrometerHeading());
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Rotation2d getHeading() {
		return poseEstimator.getEstimatedPosition().getRotation();
	}

	public Rotation2d getGyrometerHeading() {
		return gyro.getRotation2d();
	}

	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(gyro.getRotation2d(), swerve.getModulePositions(), pose);
	}

	public void resetGyrometerHeading() {
		gyro.reset();
	}
}
