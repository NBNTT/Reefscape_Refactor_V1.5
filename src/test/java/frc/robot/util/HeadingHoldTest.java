package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class HeadingHoldTest {
	private static final double TRANSLATING = 1.0;
	private static final double STOPPED = 0.0;

	@Test
	void driverRotationOverridesHold() {
		HeadingHold hold = new HeadingHold(1, 0, 0);
		assertEquals(0.5, hold.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.5), 1e-9);
		assertFalse(hold.isActive());
	}

	@Test
	void engagesWhenTranslatingWithoutRotation() {
		HeadingHold hold = new HeadingHold(1, 0, 0);
		assertEquals(0.0, hold.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0), 1e-9);
		assertTrue(hold.isActive());
	}

	@Test
	void correctsTowardCapturedHeading() {
		HeadingHold hold = new HeadingHold(1, 0, 0);
		hold.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0);
		double output = hold.update(Rotation2d.fromDegrees(10), TRANSLATING, 0.0);
		assertTrue(output < 0, "positive heading error should rotate negative");
	}

	@Test
	void releasesWhenTranslationStops() {
		HeadingHold hold = new HeadingHold(1, 0, 0);
		hold.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0);
		assertEquals(0.0, hold.update(Rotation2d.fromDegrees(10), STOPPED, 0.0), 1e-9);
		assertFalse(hold.isActive());
	}

	@Test
	void resetClearsTheTarget() {
		HeadingHold hold = new HeadingHold(1, 0, 0);
		hold.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0);
		hold.reset();
		assertFalse(hold.isActive());
		assertEquals(0.0, hold.update(Rotation2d.fromDegrees(30), TRANSLATING, 0.0), 1e-9);
	}
}
