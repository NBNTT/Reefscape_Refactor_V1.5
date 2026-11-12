package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class InputShaperTest {
	private static final double DEADBAND = 0.1;
	private static final double EXPO = 0.35;

	@Test
	void inputInsideDeadbandBecomesZero() {
		assertEquals(0.0, InputShaper.shape(0.09, DEADBAND, EXPO), 1e-9);
		assertEquals(0.0, InputShaper.shape(-0.09, DEADBAND, EXPO), 1e-9);
	}

	@Test
	void fullStickStaysFull() {
		assertEquals(1.0, InputShaper.shape(1.0, DEADBAND, EXPO), 1e-9);
		assertEquals(-1.0, InputShaper.shape(-1.0, DEADBAND, EXPO), 1e-9);
	}

	@Test
	void outputPreservesSign() {
		assertTrue(InputShaper.shape(0.5, DEADBAND, EXPO) > 0);
		assertTrue(InputShaper.shape(-0.5, DEADBAND, EXPO) < 0);
	}

	@Test
	void linearCurveIsIdentityOutsideDeadband() {
		assertEquals(0.5, InputShaper.shape(0.5, 0.0, 0.0), 1e-9);
		assertEquals(-0.25, InputShaper.shape(-0.25, 0.0, 0.0), 1e-9);
	}

	@Test
	void expoCurveReducesMidStick() {
		double linear = InputShaper.shape(0.5, 0.0, 0.0);
		double curved = InputShaper.shape(0.5, 0.0, 1.0);
		assertTrue(curved < linear);
	}
}
