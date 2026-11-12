package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Drivetrain-only robot lifecycle. */
public class Robot extends LoggedRobot {
	private final RobotContainer robotContainer;

	public Robot() {
		Logger.recordMetadata("ProjectName", "SimpleReefscape");
		Logger.recordMetadata("RuntimeMode", Constants.currentMode.toString());

		switch (Constants.currentMode) {
			case REAL -> {
				Logger.addDataReceiver(new WPILOGWriter("/U/logs/"));
				Logger.addDataReceiver(new NT4Publisher());
			}
			case SIM -> {
				new File("logs").mkdirs();
				Logger.addDataReceiver(new WPILOGWriter("logs/"));
				Logger.addDataReceiver(new NT4Publisher());
			}
		}

		Logger.start();
		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		robotContainer.resetGyro();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
}
