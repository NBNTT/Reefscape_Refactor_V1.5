package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CardinalLockConstants;

/**
 * Holds the heading captured when the driver starts translating without
 * rotating.
 */
public class CardinalLock {
	private final PIDController pid;
	private Double targetRadians;

	public CardinalLock(double kP, double kI, double kD) {
		pid = new PIDController(kP, kI, kD);
		pid.enableContinuousInput(-Math.PI, Math.PI);
	}

	/**
	 * @param currentHeading
	 *            current robot heading
	 * @param translationSpeedMps
	 *            current translation speed
	 * @param rotationInputRadPerSec
	 *            driver-commanded rotation
	 * @return angular velocity command
	 */
	public double update(Rotation2d currentHeading, double translationSpeedMps, double rotationInputRadPerSec) {
		if (Math.abs(rotationInputRadPerSec) > CardinalLockConstants.kRotationDeadband) {
			reset();
			return rotationInputRadPerSec;
		}

		if (translationSpeedMps < CardinalLockConstants.kMinTranslationSpeedMps) {
			reset();
			return 0.0;
		}

		// Capture the heading on the first translating tick, then correct back toward
		// it.
		if (targetRadians == null) {
			targetRadians = currentHeading.getRadians();
			pid.reset();
			return 0.0;
		}

		double output = pid.calculate(currentHeading.getRadians(), targetRadians);
		return MathUtil.clamp(output, -CardinalLockConstants.kMaxAngularSpeedRadPerSec,
				CardinalLockConstants.kMaxAngularSpeedRadPerSec);
	}

	public void reset() {
		targetRadians = null;
		pid.reset();
	}

	public boolean isActive() {
		return targetRadians != null;
	}
}
