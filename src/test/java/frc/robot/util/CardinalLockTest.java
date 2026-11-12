package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class CardinalLockTest {
	private static final double TRANSLATING = 1.0;
	private static final double STOPPED = 0.0;

	@Test
	void driverRotationOverridesLock() {
		CardinalLock lock = new CardinalLock(1, 0, 0);
		assertEquals(0.5, lock.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.5), 1e-9);
		assertFalse(lock.isActive());
	}

	@Test
	void engagesWhenTranslatingWithoutRotation() {
		CardinalLock lock = new CardinalLock(1, 0, 0);
		assertEquals(0.0, lock.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0), 1e-9);
		assertTrue(lock.isActive());
	}

	@Test
	void correctsTowardCapturedHeading() {
		CardinalLock lock = new CardinalLock(1, 0, 0);
		lock.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0);
		double output = lock.update(Rotation2d.fromDegrees(10), TRANSLATING, 0.0);
		assertTrue(output < 0, "positive heading error should rotate negative");
	}

	@Test
	void releasesWhenTranslationStops() {
		CardinalLock lock = new CardinalLock(1, 0, 0);
		lock.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0);
		assertEquals(0.0, lock.update(Rotation2d.fromDegrees(10), STOPPED, 0.0), 1e-9);
		assertFalse(lock.isActive());
	}

	@Test
	void resetClearsTheTarget() {
		CardinalLock lock = new CardinalLock(1, 0, 0);
		lock.update(Rotation2d.fromDegrees(0), TRANSLATING, 0.0);
		lock.reset();
		assertFalse(lock.isActive());
		assertEquals(0.0, lock.update(Rotation2d.fromDegrees(30), TRANSLATING, 0.0), 1e-9);
	}
}
