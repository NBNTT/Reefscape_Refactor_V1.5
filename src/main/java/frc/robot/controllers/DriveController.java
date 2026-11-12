package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Controller-agnostic driver input surface. Only these controls are bound. */
public interface DriveController {
	double forward();

	double strafe();

	double rotation();

	boolean slowMode();

	Trigger toggleOrientation();

	Trigger resetGyro();

	void rumble(double strength);
}
