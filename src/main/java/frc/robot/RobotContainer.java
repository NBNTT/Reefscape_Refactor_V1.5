package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.EightBitDoController;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;

/** Drivetrain wiring and bindings. */
public class RobotContainer {
	private final Swerve swerve;
	private final Odometry odometry;
	private final DriveController driver;
	private final SwerveTeleOp teleop;

	public RobotContainer() {
		swerve = new Swerve();
		odometry = new Odometry(swerve);
		driver = new EightBitDoController(OperatorConstants.kDriverControllerPort);
		teleop = new SwerveTeleOp(swerve, odometry, driver);

		swerve.setDefaultCommand(teleop);

		// Only these buttons are bound. Everything else is intentionally inert.
		driver.resetGyro().onTrue(new InstantCommand(this::resetGyro));
		driver.toggleOrientation().onTrue(new InstantCommand(teleop::toggleRobotOriented));
	}

	public void resetGyro() {
		odometry.resetGyrometerHeading();
		teleop.resetCardinalLock();
	}
}
