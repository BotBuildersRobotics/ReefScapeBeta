package frc.robot.lib.io;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

public class MotorIOTalonFXS extends MotorIO {
	protected final TalonFXS main;
	protected final TalonFXS[] followers;
	protected TalonFXSConfiguration config;
	protected TalonFXSConfiguration followerConfig;
	private final ControlRequestGetter requestGetter;
	private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
	private ThreadPoolExecutor threadPoolExecutor =
			new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

	public void applyConfig(TalonFXS fx, TalonFXSConfiguration config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				StatusCode result = fx.getConfigurator().apply(config);
				if (result.isOK()) {
					break;
				}
			}
		});
	}

	@Override
	public void updateInputs() {
		updateMotorInputs(inputs, main);

		for (int i = 0; i < followers.length; i++) {
			updateMotorInputs(followerInputs[i], followers[i]);
		}
	}

	/**
	 * Updates one Inputs from readings of a TalonFX motor
	 *
	 * @param inputsToUpdate Inputs to update from reading.
	 * @param motor Motor to read from.
	 */
	protected void updateMotorInputs(Inputs inputsToUpdate, TalonFXS motor) {
		inputsToUpdate.position = motor.getPosition().getValue();
		inputsToUpdate.velocity = motor.getVelocity().getValue();
		inputsToUpdate.statorCurrent = motor.getStatorCurrent().getValue();
		inputsToUpdate.supplyCurrent = motor.getSupplyCurrent().getValue();
		inputsToUpdate.motorVoltage = motor.getMotorVoltage().getValue();
		inputsToUpdate.motorTemperature = motor.getDeviceTemp().getValue();
	}

	private void setControl(ControlRequest request) {
		main.setControl(request);
	}

	@Override
	public void setNeutralSetpoint() {
		setControl(new NeutralOut());
	}

	@Override
	public void setCoastSetpoint() {
		setControl(new CoastOut());
	}

	@Override
	protected void setVoltageSetpoint(Voltage voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	protected void setDutyCycleSetpoint(Dimensionless percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getPositionRequest(mechanismPosition));
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			main.setPosition(mechanismPosition);
		});
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	private void setNeutralMode(TalonFXS fx, NeutralModeValue neutralMode) {
		SmartDashboard.putNumber("TALON FXS NEUTRAL MODE SET!!", Timer.getFPGATimestamp());
		threadPoolExecutor.submit(() -> {
			fx.setNeutralMode(neutralMode);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		NeutralModeValue neutralMode = wantsBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.NeutralMode = neutralMode;
		setNeutralMode(main, neutralMode);
		for (TalonFXS talon : followers) {
			setNeutralMode(talon, neutralMode);
		}
	}

	@Override
	public void useSoftLimits(boolean enable) {
		UnaryOperator<TalonFXSConfiguration> configChanger = (config) -> {
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
			return config;
		};

		changeMainConfig(configChanger);
	}

	/**
	 * Applies a TalonFXConfiguration to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setMainConfig(TalonFXSConfiguration configuration) {
		config = configuration;
		applyConfig(main, config);
	}

	/**
	 * Changes the currently applied main TalonFXConfiguration and applies the new configuration to the main motor.
	 *
	 * @param configChanger Mutating operation to apply on the current configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXSConfiguration> configChanger) {
		setMainConfig(configChanger.apply(config));
	}

	/**
	 * Applies a TalonFXConfiguration to all follower motors.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setFollowerConfig(TalonFXSConfiguration configuration) {
		followerConfig = configuration;
		for (TalonFXS talon : followers) {
			applyConfig(talon, followerConfig);
		}
	}

	/**
	 * Changes the currently applied follower TalonFXConfiguration and applies the new configuration to all follower motors.
	 *
	 * @param configChanger Mutating operation to apply on the current configuration.
	 */
	public void changeFollowerConfig(UnaryOperator<TalonFXSConfiguration> configChanger) {
		setFollowerConfig(configChanger.apply(followerConfig));
	}

	/**
	 * Creates a MotorIOTalonFX from a provided configuration.
	 *
	 * @param config Configuration to create MotorIOTalonFX from.
	 */
	public MotorIOTalonFXS(MotorIOTalonFXSConfig config) {
		super(config.unit, config.time, config.followerIDs.length);
		requestGetter = config.requestGetter;
		main = new TalonFXS(config.mainID, config.mainBus);
		setMainConfig(config.mainConfig);

		followers = new TalonFXS[config.followerIDs.length];
		for (int i = 0; i < config.followerIDs.length; i++) {
			followers[i] = new TalonFXS(config.followerIDs[i], config.followerBuses[i]);
			followers[i].setControl(new Follower(config.mainID, config.followerOpposeMain[i]));
		}

		setFollowerConfig(followerConfig);
	}


	public static class MotorIOTalonFXSConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		public int mainID = -1;
		public String mainBus = "ASSIGN_BUS";
		public TalonFXSConfiguration mainConfig = new TalonFXSConfiguration();
		public int[] followerIDs = new int[0];
		public String[] followerBuses = new String[0];
		public boolean[] followerOpposeMain = new boolean[0];
		public ControlRequestGetter requestGetter = new ControlRequestGetter();
	}

	public static class ControlRequestGetter {
		public ControlRequest getVoltageRequest(Voltage voltage) {
			return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
		}

		public ControlRequest getDutyCycleRequest(Dimensionless percent) {
			return new DutyCycleOut(percent.in(Units.Percent));
		}

		public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(Angle mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}
	}
}