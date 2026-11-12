package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.Test;

class SwerveFailureTest {
	private static final class FakeModule implements SwerveModuleIO {
		boolean healthy = true;
		boolean coasted = false;
		boolean recovered = false;
		SwerveModulePosition position = new SwerveModulePosition(0, new Rotation2d());
		SwerveModuleState state = new SwerveModuleState();

		@Override
		public void setDesiredState(SwerveModuleState desiredState) {
		}

		@Override
		public SwerveModuleState getState() {
			return state;
		}

		@Override
		public SwerveModulePosition getPosition() {
			return position;
		}

		@Override
		public double getAbsoluteEncoderAngle() {
			return 0;
		}

		@Override
		public boolean isHealthy() {
			return healthy;
		}

		@Override
		public void setCoast() {
			coasted = true;
		}

		@Override
		public void setBrake() {
		}

		@Override
		public void recover(SwerveModulePosition lastGoodPosition) {
			position = lastGoodPosition;
			recovered = true;
		}

		@Override
		public void resetEncoders() {
		}

		@Override
		public void stop() {
		}
	}

	private static Swerve swerveWith(FakeModule frontLeft) {
		return new Swerve(new SwerveModuleIO[]{frontLeft, new FakeModule(), new FakeModule(), new FakeModule()});
	}

	@Test
	void failedModuleIsCoastedAndFrozen() {
		FakeModule frontLeft = new FakeModule();
		Swerve swerve = swerveWith(frontLeft);
		SwerveModuleState lastGood = swerve.getModuleStates()[0];

		frontLeft.healthy = false;
		swerve.periodic();

		assertTrue(frontLeft.coasted);
		assertSame(lastGood, swerve.getModuleStates()[0]);
	}

	@Test
	void recoversAfterStableReconnect() throws InterruptedException {
		FakeModule frontLeft = new FakeModule();
		Swerve swerve = swerveWith(frontLeft);

		frontLeft.healthy = false;
		swerve.periodic();
		assertTrue(frontLeft.coasted);

		frontLeft.healthy = true;
		swerve.periodic();
		Thread.sleep(600);
		swerve.periodic();

		assertTrue(frontLeft.recovered);
	}
}
