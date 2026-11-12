package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HeadingHoldConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.util.HeadingHold;
import frc.robot.util.InputShaper;
import java.util.function.Supplier;

/**
 * Driver teleop: input shaping, heading hold, and field/robot-oriented drive.
 */
public class SwerveTeleOp extends Command {
	private final Swerve swerve;
	private final Odometry odometry;
	private final CommandXboxController driver;
	private final Supplier<Double> xSpdFunction;
	private final Supplier<Double> ySpdFunction;
	private final Supplier<Double> turningSpdFunction;
	private final Supplier<Boolean> slowMode;
	private final SlewRateLimiter xLimiter;
	private final SlewRateLimiter yLimiter;
	private final SlewRateLimiter turningLimiter;
	private final HeadingHold headingHold;

	private boolean robotOriented = false;

	public SwerveTeleOp(Swerve swerve, Odometry odometry, CommandXboxController driver, Supplier<Double> xSpdFunction,
			Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> slowMode) {
		this.swerve = swerve;
		this.odometry = odometry;
		this.driver = driver;
		this.xSpdFunction = xSpdFunction;
		this.ySpdFunction = ySpdFunction;
		this.turningSpdFunction = turningSpdFunction;
		this.slowMode = slowMode;

		xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
		headingHold = new HeadingHold(HeadingHoldConstants.kP, HeadingHoldConstants.kI, HeadingHoldConstants.kD);

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		headingHold.reset();
	}

	@Override
	public void execute() {
		double xSpeed = InputShaper.shape(xSpdFunction.get(), InputConstants.kTranslationDeadband,
				InputConstants.kExpo);
		double ySpeed = InputShaper.shape(ySpdFunction.get(), InputConstants.kTranslationDeadband,
				InputConstants.kExpo);
		double turningSpeed = InputShaper.shape(turningSpdFunction.get(), InputConstants.kRotationDeadband,
				InputConstants.kExpo);

		if (slowMode.get()) {
			xSpeed *= InputConstants.kSlowModeScale;
			ySpeed *= InputConstants.kSlowModeScale;
			turningSpeed *= InputConstants.kSlowModeScale;
			driver.getHID().setRumble(RumbleType.kRightRumble, 0.025);
		} else {
			driver.getHID().setRumble(RumbleType.kBothRumble, 0);
		}

		xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		turningSpeed = turningLimiter.calculate(turningSpeed)
				* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

		turningSpeed = headingHold.update(odometry.getGyrometerHeading(), Math.hypot(xSpeed, ySpeed), turningSpeed);

		ChassisSpeeds chassisSpeeds = robotOriented
				? new ChassisSpeeds(xSpeed, ySpeed, turningSpeed)
				: ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, odometry.getGyrometerHeading());

		swerve.setModuleStates(chassisSpeeds);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stopModules();
		headingHold.reset();
	}

	public void toggleRobotOriented() {
		robotOriented = !robotOriented;
		headingHold.reset();
	}

	public void resetHeadingHold() {
		headingHold.reset();
	}

	public boolean isRobotOriented() {
		return robotOriented;
	}
}
