package frc.robot.subsystems;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.FieldLayout.Level;
import frc.robot.lib.io.BeamBreakIO;
import frc.robot.lib.io.BeamBreakIOCANRange;
import frc.robot.lib.io.BeamBreakIOSim;
import frc.robot.subsystems.drive.ControlBoardConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

import frc.robot.Ports;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;


public class SuperSystemConstants {
    public static class BeamBreakConstants 
    {
        
        public static BeamBreakIO getIndexerBeamBreak() {
			
            CANrangeConfiguration config = new CANrangeConfiguration();

            return new BeamBreakIOCANRange(
                Ports.INTAKE_1_CANRANGE.getDeviceNumber(), 
                Ports.INTAKE_1_CANRANGE.getBus(),
                config,
                SuperSystemConstants.kIntakeCoralBeamBreakDebounce,
                "Intake Beam Break", 100);
			
		}

       

        
    }

    public static Distance getGamepieceOffsetFactor(Level level) {
        return switch (level) {
            case L4 -> SuperSystemConstants.kL4CoralOffsetFactor;
            case L3 -> SuperSystemConstants.kL3CoralOffsetFactor;
            case L2 -> SuperSystemConstants.kL2CoralOffsetFactor;
            case L1 -> SuperSystemConstants.kL1CoralOffsetFactor;
            case L3_ALGAE, L2_ALGAE -> SuperSystemConstants.kAlgaeOffsetFactor;
            case ALGAE_READY -> SuperSystemConstants.kAlgaeReadyOffsetFactor;
            default -> kL4CoralOffsetFactor;
        };
    }

    public static Time getAutoAlignScoringDelay(Level level) {
        Time delay = Units.Milliseconds.of(100);
         return switch (level) {
             case L1 -> delay;
             case L2 -> delay;
             case L3 -> delay;
             case L4 -> delay;
             case PROCESSOR_ALGAE -> delay;
             case NET -> delay;
             default -> delay;
         };
     }

    public static Distance getAutoAlignScoringDistanceEpsilon(Level level) {
		return switch (level) {
			case L1 -> kL1ScoringDistanceEpsilon;
			case L2 -> kL2ScoringDistanceEpsilon;
			case L3 -> kL3ScoringDistanceEpsilon;
			case L4 -> kL4ScoringDistanceEpsilon;
			case PROCESSOR_ALGAE -> kProcessorAlgaeScoringDistanceEpsilon;
			case NET -> kNetScoringDistanceEpsilon;
			default -> kReefScoringDistanceEpsilon;
		};
	}

    public static Angle getAutoAlignScoringAngleEpsilon(Level level) {
		return switch (level) {
			case L1 -> kL1ScoringAngleEpsilon;
			case L2 -> kL2ScoringAngleEpsilon;
			case L3 -> kL3ScoringAngleEpsilon;
			case L4 -> kL4ScoringAngleEpsilon;
			case PROCESSOR_ALGAE -> kProcessorAlgaeScoringAngleEpsilon;
			case NET -> kNetScoringAngleEpsilon;
			default -> kReefScoringAngleEpsilon;
		};
	}

    public static Angle getAutoAlignHeadingGenerationDeadband(Level level) {
		return switch (level) {
			case L2 -> kL2HeadingGenerationDeadband;
			case L3 -> kL3HeadingGenerationDeadband;
			case L4 -> kL4HeadingGenerationDeadband;
			case PROCESSOR_ALGAE -> kProcessorAlgaeHeadingGenerationDeadband;
			default -> kReefHeadingGenerationDeadband;
		};
	}

    public static final Time kIntakelRollersCurrentSpikeDebounce = Units.Seconds.of(0.4);
    public static final Time kIntakeRollersVelocityDebounce = Units.Seconds.of(0.04);
    public static final Time kIntakeCoralBeamBreakDebounce = Units.Seconds.of(0.04);

    public static final Distance kElevatorCenterOffset = Units.Inches.of(12.5);
    public static final Time lookaheadBranchSelectionTime = Units.Milliseconds.of(100.0);

    public static final Distance kAlgaeOffsetFactor = Units.Centimeters.of(10.0);
	public static final Distance kAlgaeReadyOffsetFactor = Units.Centimeters.of(20.0);
	public static final Distance kL4CoralOffsetFactor = Units.Centimeters.of(34.25);
	public static final Distance kL3CoralOffsetFactor = Units.Centimeters.of(30.25);
	public static final Distance kL2CoralOffsetFactor = Units.Centimeters.of(30.25);
	public static final Distance kL1CoralOffsetFactor = Units.Centimeters.of(60.0);

    public static final Distance kReefScoringDistanceEpsilon = Units.Centimeters.of(3.5);
	public static final Distance kProcessorAlgaeScoringDistanceEpsilon = Units.Centimeters.of(12.0);
	public static final Distance kL1ScoringDistanceEpsilon = Units.Centimeters.of(3.5);
	public static final Distance kL2ScoringDistanceEpsilon = Units.Centimeters.of(3.0);
	public static final Distance kL3ScoringDistanceEpsilon = Units.Centimeters.of(3.0);
	public static final Distance kL4ScoringDistanceEpsilon = Units.Centimeters.of(3.0);
	public static final Distance kNetScoringDistanceEpsilon = Units.Centimeters.of(8.0);

    public static final Angle kReefScoringAngleEpsilon = Units.Degrees.of(0.8);
	public static final Angle kProcessorAlgaeScoringAngleEpsilon = Units.Degrees.of(3.0);
	public static final Angle kL1ScoringAngleEpsilon = Units.Degrees.of(2.0);
	public static final Angle kL2ScoringAngleEpsilon = Units.Degrees.of(1.2);
	public static final Angle kL3ScoringAngleEpsilon = Units.Degrees.of(1.2);
	public static final Angle kL4ScoringAngleEpsilon = Units.Degrees.of(1.2);
	public static final Angle kNetScoringAngleEpsilon = Units.Degrees.of(4.2);

    public static final Angle kReefHeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kProcessorAlgaeHeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL1HeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL2HeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL3HeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL4HeadingGenerationDeadband = Units.Degrees.of(0.0);

    public static final Time kRecentUpdateTime = Units.Seconds.of(0.1);
	public static final Distance kNearUpdateDistance = Units.Centimeters.of(2.0);
    public static final Distance kL1CoralHorizontalOffsetFactor = Units.Inches.of(6.469);


}
