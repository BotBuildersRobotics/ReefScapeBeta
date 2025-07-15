package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.io.ServoMotorSubsystem.ServoHomingConfig;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.lib.Util;
import frc.robot.Ports;
import com.ctre.phoenix6.signals.InvertedValue;
import static edu.wpi.first.units.Units.*;

public class ElevatorConstants {

	public static final double kGearing = 6.285 * 4;

	//public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(
	//		Units.Inches.of(2.0).plus(Units.Inches.of(0.125)).div(2.0));
	
	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(Units.Centimeters.of(2.0));
	
	

	public static final Distance kL1ScoringHeight = Units.Centimeters.of(50);
	
	public static final Distance kL2ScoringHeight = Units.Centimeters.of(6);

	public static final Distance kL2AlgaeHeight = Units.Centimeters.of(40);


	public static final Distance kL3ScoringHeight = Units.Centimeters.of(26 );
	
	
	public static final Distance kLIntakeHeight = Units.Centimeters.of(13);
	
	
	public static final Distance kStowPosition = Units.Centimeters.of(0.5);

	public static final Distance kEpsilonThreshold = Units.Centimeters.of(5.0);

	
	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = 39.6;
		
		FXConfig.Slot0.kV = 0.2;
      
		FXConfig.Slot0.kG = 0.66; //volts to overcome gravity
      
		FXConfig.Slot0.kS = 0.2;// volts to get over the static friction
    
		FXConfig.Slot0.kA = 0.02; //volts for accel 

		FXConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		MotionMagicConfigs mm = FXConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(60)) 
		.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80))
		.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(220));

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		FXConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 60.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		//FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		//FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 16;

		//FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		//FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =0;

		FXConfig.Feedback.SensorToMechanismRatio = kGearing;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = Ports.ELEVATOR.getDeviceNumber();
		IOConfig.mainBus = Ports.ELEVATOR.getBus();
		IOConfig.unit = converter.getDistanceUnitAsAngleUnit(Units.Centimeters);
		IOConfig.time = Units.Second;
		
		return IOConfig;
	}

	

	public static MotorIOTalonFX getMotorIO() {
		
			return new MotorIOTalonFX(getIOConfig());
		 
	}

	
}