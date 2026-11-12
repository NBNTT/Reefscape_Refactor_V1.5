package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CardinalLockConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.controllers.DriveController;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.util.CardinalLock;
import frc.robot.util.InputShaper;
import org.littletonrobotics.junction.Logger;

/**
 * Driver teleop: cubic shaping, Cardinal Lock, and field/robot-oriented drive.
 */
public class SwerveTeleOp extends Command {
	private final Swerve swerve;
	private final Odometry odometry;
	private final DriveController driver;
	private final SlewRateLimiter xLimiter;
	private final SlewRateLimiter yLimiter;
	private final SlewRateLimiter turningLimiter;
	private final CardinalLock cardinalLock;

	private boolean robotOriented = false;

	public SwerveTeleOp(Swerve swerve, Odometry odometry, DriveController driver) {
		this.swerve = swerve;
		this.odometry = odometry;
		this.driver = driver;

		xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
		cardinalLock = new CardinalLock(CardinalLockConstants.kP, CardinalLockConstants.kI, CardinalLockConstants.kD);

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		cardinalLock.reset();
	}

	@Override
	public void execute() {
		if (!DriverStation.isEnabled()) {
			swerve.stopModules();
			cardinalLock.reset();
			return;
		}

		double xSpeed = InputShaper.shape(driver.forward(), InputConstants.kTranslationDeadband);
		double ySpeed = InputShaper.shape(driver.strafe(), InputConstants.kTranslationDeadband);
		double turningSpeed = InputShaper.shape(driver.rotation(), InputConstants.kRotationDeadband);

		if (driver.slowMode()) {
			xSpeed *= InputConstants.kSlowModeScale;
			ySpeed *= InputConstants.kSlowModeScale;
			turningSpeed *= InputConstants.kSlowModeScale;
			driver.rumble(0.025);
		} else {
			driver.rumble(0.0);
		}

		xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		turningSpeed = turningLimiter.calculate(turningSpeed)
				* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

		turningSpeed = cardinalLock.update(odometry.getGyrometerHeading(), Math.hypot(xSpeed, ySpeed), turningSpeed);

		ChassisSpeeds chassisSpeeds = robotOriented
				? new ChassisSpeeds(xSpeed, ySpeed, turningSpeed)
				: ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, odometry.getGyrometerHeading());

		swerve.setModuleStates(chassisSpeeds);

		Logger.recordOutput("Drive/CardinalLocked", cardinalLock.isActive());
		Logger.recordOutput("Drive/RobotOriented", robotOriented);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stopModules();
		cardinalLock.reset();
	}

	public void toggleRobotOriented() {
		robotOriented = !robotOriented;
		cardinalLock.reset();
	}

	public void resetCardinalLock() {
		cardinalLock.reset();
	}

	public boolean isRobotOriented() {
		return robotOriented;
	}
}
