package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * 8BitDo Ultimate Wireless in XInput mode. It presents as an Xbox controller,
 * so the standard left/right stick and button mapping applies.
 */
public class EightBitDoController implements DriveController {
	private final CommandXboxController controller;

	public EightBitDoController(int port) {
		controller = new CommandXboxController(port);
	}

	@Override
	public double forward() {
		return -controller.getLeftY();
	}

	@Override
	public double strafe() {
		return -controller.getLeftX();
	}

	@Override
	public double rotation() {
		return -controller.getRightX();
	}

	@Override
	public boolean slowMode() {
		return controller.rightBumper().getAsBoolean();
	}

	@Override
	public Trigger toggleOrientation() {
		return controller.y();
	}

	@Override
	public Trigger resetGyro() {
		return controller.back();
	}

	@Override
	public void rumble(double strength) {
		controller.getHID().setRumble(RumbleType.kBothRumble, strength);
	}
}
