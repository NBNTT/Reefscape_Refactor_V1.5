package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class InputShaperTest {
	private static final double DEADBAND = 0.1;

	@Test
	void inputInsideDeadbandBecomesZero() {
		assertEquals(0.0, InputShaper.shape(0.09, DEADBAND), 1e-9);
		assertEquals(0.0, InputShaper.shape(-0.09, DEADBAND), 1e-9);
	}

	@Test
	void fullStickStaysFull() {
		assertEquals(1.0, InputShaper.shape(1.0, DEADBAND), 1e-9);
		assertEquals(-1.0, InputShaper.shape(-1.0, DEADBAND), 1e-9);
	}

	@Test
	void outputPreservesSign() {
		assertTrue(InputShaper.shape(0.5, DEADBAND) > 0);
		assertTrue(InputShaper.shape(-0.5, DEADBAND) < 0);
	}

	@Test
	void responseIsCubic() {
		assertEquals(0.125, InputShaper.shape(0.5, 0.0), 1e-9);
		assertEquals(-0.125, InputShaper.shape(-0.5, 0.0), 1e-9);
	}
}
