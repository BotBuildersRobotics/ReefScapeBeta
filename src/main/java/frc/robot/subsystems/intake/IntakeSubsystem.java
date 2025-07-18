package frc.robot.subsystems.intake;

import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFXS;


public class IntakeSubsystem extends MotorSubsystem<MotorIOTalonFXS> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(IntakeConstants.kExhaustVoltage);
	public static final Setpoint SCORE = Setpoint.withVoltageSetpoint(IntakeConstants.kScoreVoltage);
	public static final Setpoint AUTO_SCORE = Setpoint.withVoltageSetpoint(IntakeConstants.kScoreAutoVoltage);
	

	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(IntakeConstants.kReverseVoltage);
	public static final Setpoint SLOW_INTAKE = Setpoint.withVoltageSetpoint(IntakeConstants.kSlowIntakeVoltage);

	public static final IntakeSubsystem mInstance = new IntakeSubsystem();

	public IntakeSubsystem() {
		super(IntakeConstants.getMotorIO(), "Intake Rollers");
	}

}