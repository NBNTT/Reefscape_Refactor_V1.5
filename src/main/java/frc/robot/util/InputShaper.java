package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/** Pure joystick input shaping: deadband, expo/cubic, clamp. */
public final class InputShaper {
	private InputShaper() {
	}

	/**
	 * Applies a deadband and an expo curve while preserving sign and full-scale
	 * output.
	 *
	 * @param value
	 *            raw joystick value
	 * @param deadband
	 *            values with magnitude at or below this become zero
	 * @param expo
	 *            0 = linear, 1 = cubic
	 */
	public static double shape(double value, double deadband, double expo) {
		double shaped = MathUtil.clamp(value, -1.0, 1.0);
		shaped = MathUtil.applyDeadband(shaped, deadband);

		double curve = MathUtil.clamp(expo, 0.0, 1.0);
		double magnitude = Math.abs(shaped);
		double output = curve * Math.pow(magnitude, 3) + (1.0 - curve) * magnitude;
		return Math.copySign(output, shaped);
	}
}
