package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/** Pure joystick input shaping. */
public final class InputShaper {
	private InputShaper() {
	}

	/**
	 * Applies a deadband, then a cubic response. Cubic keeps small corrections
	 * small while preserving full output at full stick.
	 */
	public static double shape(double value, double deadband) {
		double limited = MathUtil.applyDeadband(MathUtil.clamp(value, -1.0, 1.0), deadband);
		return limited * limited * limited;
	}
}
