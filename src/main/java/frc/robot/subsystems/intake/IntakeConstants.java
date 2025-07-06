package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.lib.io.MotorIOTalonFXS;
import frc.robot.Ports;
import frc.robot.Robot;


public class IntakeConstants {
	private static final double kGearing = (24.0 / 12.0);

	public static final Voltage kStartVoltage = Units.Volts.of(3.0);
	public static final Voltage kIntakeVoltage = Units.Volts.of( -12.0);
	
	public static final Voltage kExhaustVoltage = Units.Volts.of(12.0);

	public static TalonFXSConfiguration getFXConfig() {
		TalonFXSConfiguration config = new TalonFXSConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFXS.MotorIOTalonFXSConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFXS.MotorIOTalonFXSConfig config = new frc.robot.lib.io.MotorIOTalonFXS.MotorIOTalonFXSConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.INTAKE.getDeviceNumber();
		config.mainBus = Ports.INTAKE.getBus();
		return config;
	}

	public static MotorIOTalonFXS getMotorIO() {
		
		return new MotorIOTalonFXS(getIOConfig());
		
	}

}