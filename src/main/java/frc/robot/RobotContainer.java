package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;

/** Drivetrain wiring and bindings. */
public class RobotContainer {
	private final Swerve swerve;
	private final Odometry odometry;
	private final CommandXboxController driver;
	private final SwerveTeleOp teleop;

	public RobotContainer() {
		swerve = new Swerve();
		odometry = new Odometry(swerve);
		driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);

		teleop = new SwerveTeleOp(swerve, odometry, driver, () -> -driver.getLeftY(), () -> -driver.getLeftX(),
				() -> -driver.getRightX(), () -> driver.rightBumper().getAsBoolean());

		swerve.setDefaultCommand(teleop);

		driver.back().onTrue(new InstantCommand(odometry::resetGyrometerHeading));
		driver.y().onTrue(new InstantCommand(teleop::toggleRobotOriented));
	}

	public void resetGyro() {
		odometry.resetGyrometerHeading();
		teleop.resetHeadingHold();
	}
}
