package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** mk4n REV implementation of one swerve module. */
public class SwerveModule implements SwerveModuleIO {
	private final SparkMax driveMotor;
	private final SparkMax turningMotor;
	private final SparkMaxConfig driveConfig;
	private final SparkMaxConfig turningConfig;
	private final PIDController turningPidController;
	private final AnalogInput absoluteEncoder;
	private final double offsetRadians;
	private final boolean reversed;

	/**
	 * Keeps odometry continuous after a module reconnects and its drive encoder is
	 * re-seeded.
	 */
	private double positionOffsetMeters;

	public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
			int absoluteEncoderId, double offsetRadians, boolean reversed) {
		this.offsetRadians = offsetRadians;
		this.reversed = reversed;

		absoluteEncoder = new AnalogInput(absoluteEncoderId);
		driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
		turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

		driveConfig = new SparkMaxConfig();
		driveConfig.voltageCompensation(12.0);
		driveConfig.smartCurrentLimit(40);
		driveConfig.inverted(driveMotorReversed);
		driveConfig.idleMode(IdleMode.kBrake);
		driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
		driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

		turningConfig = new SparkMaxConfig();
		turningConfig.voltageCompensation(12.0);
		turningConfig.smartCurrentLimit(40);
		turningConfig.inverted(turningMotorReversed);
		turningConfig.idleMode(IdleMode.kBrake);
		turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
		turningConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

		driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
		turningPidController.enableContinuousInput(-Math.PI, Math.PI);

		resetEncoders();
	}

	private double getDrivePosition() {
		return driveMotor.getEncoder().getPosition() + positionOffsetMeters;
	}

	private double getDriveVelocity() {
		return driveMotor.getEncoder().getVelocity();
	}

	private double getTurningPosition() {
		return getAbsoluteEncoderAngle();
	}

	@Override
	public double getAbsoluteEncoderAngle() {
		double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
		angle *= 2 * Math.PI;
		angle += offsetRadians;
		angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
		return angle * (reversed ? -1 : 1);
	}

	@Override
	public boolean isHealthy() {
		var driveFaults = driveMotor.getFaults();
		var turningFaults = turningMotor.getFaults();
		return !driveFaults.can && !driveFaults.sensor && !turningFaults.can && !turningFaults.sensor;
	}

	private void setIdleMode(IdleMode mode) {
		driveConfig.idleMode(mode);
		turningConfig.idleMode(mode);
		driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void setCoast() {
		setIdleMode(IdleMode.kCoast);
	}

	@Override
	public void setBrake() {
		setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void recover(SwerveModulePosition lastGoodPosition) {
		// Start the recovered module where odometry last trusted it, so the pose does
		// not jump.
		positionOffsetMeters = lastGoodPosition.distanceMeters;
		driveMotor.getEncoder().setPosition(0);
		turningMotor.getEncoder().setPosition(getAbsoluteEncoderAngle());
		setBrake();
		driveMotor.clearFaults();
		turningMotor.clearFaults();
	}

	@Override
	public void resetEncoders() {
		positionOffsetMeters = 0;
		driveMotor.getEncoder().setPosition(0);
		turningMotor.getEncoder().setPosition(getAbsoluteEncoderAngle());
	}

	@Override
	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
	}

	@Override
	public void setDesiredState(SwerveModuleState desiredState) {
		if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}

		desiredState.optimize(getState().angle);
		driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
		turningMotor.set(turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
	}

	@Override
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(), getState().angle);
	}

	@Override
	public void stop() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}
}
