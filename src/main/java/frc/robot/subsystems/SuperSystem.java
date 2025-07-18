package frc.robot.subsystems;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

import frc.robot.lib.FieldLayout;
import frc.robot.lib.FieldLayout.Branch;
import frc.robot.lib.FieldLayout.Branch.Face;
import frc.robot.lib.FieldLayout.Level;
import frc.robot.lib.drive.AutoAlignPID2;
import frc.robot.lib.io.BeamBreakIO;
import frc.robot.subsystems.SuperSystemConstants.BeamBreakConstants;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;



public class SuperSystem extends SubsystemBase {
    
   
    public static BeamBreakIO indexerBeamBrake = BeamBreakConstants.getIndexerBeamBreak();
	

    public static SuperSystem mInstance;

    private Branch targetingBranch = Branch.A;
    private Face targetingFace = targetingBranch.getKey().face();

    private boolean isPathFollowing = false;

	private boolean hasAlgae = false;

    public boolean readyToRaiseElevator = false;

    private boolean targetingL3ReefIntake = true;


    public static SuperSystem getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new SuperSystem();
		}
		return mInstance;
	}

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
        indexerBeamBrake.initSendable(builder);

       
		builder.addStringProperty("Targeting Branch", () -> targetingBranch.toString(), null);
		
        builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);

    }

    @Override
	public void periodic() {
		if (!isPathFollowing) {
			updateTargetedBranch();
			updateTargetedFace();
			updateTargetedReefIntake();
		}
	}

    public void updateTargetedBranch() {
		SwerveDriveState currentState = DriveSubsystem.mInstance.getState();
		Transform2d speedsPose = new Transform2d(
						currentState.Speeds.vxMetersPerSecond,
						currentState.Speeds.vyMetersPerSecond,
						Rotation2d.fromRadians(currentState.Speeds.omegaRadiansPerSecond))
				.times(SuperSystemConstants.lookaheadBranchSelectionTime.in(Units.Seconds));
		Pose2d lookeaheadPose = currentState.Pose.transformBy(speedsPose);
		targetingBranch = FieldLayout.Branch.getClosestBranch(lookeaheadPose, RobotConstants.isRedAlliance);
	}

    public void updateTargetedFace() {
		targetingFace = targetingBranch.getKey().face();
	}

	public void updateTargetedReefIntake() {
		targetingL3ReefIntake = switch (targetingFace) {
			case NEAR_CENTER, FAR_LEFT, FAR_RIGHT -> true;
			case FAR_CENTER, NEAR_LEFT, NEAR_RIGHT -> false;};
	}


    public Command idleIntakes() {
		
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE);
					
	}

	public Command slowIntake() {
		
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.SLOW_INTAKE);
					
	}

	public Command Score(){
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.SCORE);
		
	}

	public Command ScoreAutoL1(){
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.AUTO_SCORE);
	}


	public Command ScoreL2() {

		return Commands.sequence(
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.L2_SCORE),
			Commands.waitSeconds(0.3),
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.SCORE),
			Commands.waitSeconds(1),
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.STOW),
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE)
		);

	}

	public Command ScoreAutoL2() {

		return Commands.sequence(
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.L2_SCORE),
			Commands.waitSeconds(0.3),
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.AUTO_SCORE),
			Commands.waitSeconds(1),
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.STOW),
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE)
		);

	}

	public Command ScoreL3() {

		return Commands.sequence(
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.L3_SCORE),
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.SCORE),
			Commands.waitSeconds(1),
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.STOW),
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE)
		);

	}

	public Command Intake(){
		return Commands.sequence(
                     
		
			Commands.either(
					
					IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE),
					IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.INTAKE),
					
					() -> indexerBeamBrake.getDebouncedIfReal()
			)
					
					
			)
			.withDeadline(indexerBeamBrake.stateWaitWithDebounceIfReal(true, 1.5))
			.andThen(
				IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE)
			);
		
	}

	public Command BeamBreak(){
		return indexerBeamBrake.stateWaitWithDebounceIfReal(true, 0);
	}

	public Command L2ScorePos(){
		return ElevatorSubsystem.mInstance.setpointCommand(ElevatorSubsystem.L2_SCORE);
	}

	public Command L3ScorePos(){
		return ElevatorSubsystem.mInstance.setpointCommand(ElevatorSubsystem.L3_SCORE);
	}

	public Command reverseIntake(){
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.REVERSE);
	}

	public Command StowSlides(){
		return Commands.sequence(
			ElevatorSubsystem.mInstance.setpointCommandWithWait(ElevatorSubsystem.STOW)
			
		);
		
	}

	public Command autoAlign(BooleanSupplier rightSide)
	{
		return Commands.sequence(Commands.defer(
				() -> {
					return new AutoAlignPID2(DriveSubsystem.mInstance, rightSide);
				},
				Set.of(DriveSubsystem.mInstance)));
		
	}

	//Manual Raising of Elevator to bump coral up
	//public Command ElevatorUp() {

		//return Commands.runOnce(() -> ElevatorSubsystem.mInstance.applySetpoint(ElevatorSubsystem.JOG_UP));

	//}


    public Command L1Elevator(){
        return  ElevatorSubsystem.mInstance.setpointCommand(ElevatorSubsystem.L1_SCORE);
    }

    public Command HomeElevator(){
        return Commands.sequence(
		
          
            ElevatorSubsystem.mInstance.setpointCommand(ElevatorSubsystem.STOW)
        );
    }





	
	


    public void setPathFollowing(boolean following){
		isPathFollowing = following;
    }

	public Branch getTargetingBranch() {
		return targetingBranch;
	}

	public Face getTargetingFace() {
		return targetingFace;
	}



}
