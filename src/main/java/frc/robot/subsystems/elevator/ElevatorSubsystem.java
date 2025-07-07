package frc.robot.subsystems.elevator;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.ServoMotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;

public class ElevatorSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {
	
	public static final Setpoint JOG_UP = Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(0.5, Units.Volts));
	public static final Setpoint JOG_DOWN = Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(-0.5, Units.Volts));
	
	
	
	public static final Setpoint L3_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL3ScoringHeight));
	
	

	public static final Setpoint L2_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL2ScoringHeight));
	
    public static final Setpoint L1_SCORE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL1ScoringHeight));
	

	public static final Setpoint L2_ALGAE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL2AlgaeHeight));

    public static final Setpoint STOW =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition));
	
	public static final Setpoint INTAKE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kLIntakeHeight));
			
	public static final ElevatorSubsystem mInstance = new ElevatorSubsystem();

	private ElevatorSubsystem() {
		super(
				ElevatorConstants.getMotorIO(),
				"Elevator",
				ElevatorConstants.converter.toAngle(ElevatorConstants.kEpsilonThreshold),
				ElevatorConstants.getServoConfig());
		setCurrentPosition(ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition));
		applySetpoint(STOW);
	}

	
}