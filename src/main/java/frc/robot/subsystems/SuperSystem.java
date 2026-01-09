package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.BooleanSupplier;


import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConstants;
import frc.robot.lib.FieldLayout;
import frc.robot.lib.FieldLayout.Branch;
import frc.robot.lib.FieldLayout.Branch.Face;
import frc.robot.lib.drive.DriveToPose;
import frc.robot.lib.io.BeamBreakIO;
import frc.robot.subsystems.SuperSystemConstants.BeamBreakConstants;
import frc.robot.subsystems.drive.AutoPilotTest;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;



public class SuperSystem extends SubsystemBase {
    
   
    public static BeamBreakIO indexerBeamBrake = BeamBreakConstants.getIndexerBeamBreak();
	

    public static SuperSystem mInstance;

    private Branch targetingBranch = Branch.A;
    private Face targetingFace = targetingBranch.getKey().face();

    private boolean isPathFollowing = false;

	private boolean hasAlgae = false;

    public boolean readyToRaiseElevator = false;

    private boolean targetingL3ReefIntake = true;

	private DriveToPose driveToPose;

	private AprilTagFieldLayout kAprilTagMap = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);


    public static SuperSystem getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new SuperSystem();
		}
		return mInstance;
	}

	public SuperSystem(){
		driveToPose = new DriveToPose(DriveSubsystem.mInstance);
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
			SmartDashboard.putString("Found Face", targetingFace.name());
			SmartDashboard.putString("Found Branch", targetingBranch.name());
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

	 private final NetworkTable autoPilotTable = NetworkTableInstance.getDefault().getTable("AutoPilot");
   
	private final ProtobufPublisher<Pose2d> targetPose2d = autoPilotTable
            .getProtobufTopic("AutoPilot Target Pose CMD", Pose2d.proto).publish();
	
	

	public Command AutoPilotTest2(BooleanSupplier rightSide){
		
		double tagid = LimelightHelpers.getLimelightNTDouble("", "tid");//.getLatestResults("limelight");
		RawFiducial[] rawData = LimelightHelpers.getRawFiducials("limelight");
		if(rawData == null || rawData.length == 0){
			SmartDashboard.putBoolean("valid tags",false);
			return Commands.none();
		}
		
		Pose2d closest = kAprilTagMap.getTagPose((int)tagid).get().toPose2d();
		Distance inOutDist = Units.Meters.of(0.5);
		Distance leftRightDist = rightSide.getAsBoolean() ? Units.Meters.of(0.2) : Units.Meters.of(-0.1);
		Transform2d distAwayTransform = new Transform2d(inOutDist,leftRightDist, Rotation2d.fromDegrees( 180));
		closest = closest.transformBy(distAwayTransform);
		

		
		targetPose2d.accept(closest);
		

		SmartDashboard.putNumber("Target Branch Angle", closest.getRotation().getDegrees());

		AutoPilotTest apc = new AutoPilotTest(DriveSubsystem.mInstance.getDrivetrain(), closest, Rotation2d.kZero);

		return Commands.deferredProxy(() ->apc);
	}

	private static final APConstraints kConstraints = new APConstraints()
    .withAcceleration(5.0)
    .withJerk(2.0);

	private static final APProfile kProfile = new APProfile(kConstraints)
    .withErrorXY(Units.Centimeters.of(2))
    .withErrorTheta(Units.Degrees.of(0.5))
    .withBeelineRadius(Units.Centimeters.of(8));

	public static final Autopilot kAutopilot = new Autopilot(kProfile);

	private SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    .withDriveRequestType(DriveRequestType.Velocity)
    .withHeadingPID(4, 0, 0); /* change these values for your robot */

	public Command APAlign()
	{

		Branch closestBranch =  FieldLayout.Branch.getClosestBranch(DriveSubsystem.mInstance.getPose(), true);

		 
		Pose2d closest = FieldLayout.getCoralScoringPose( closestBranch);

		Distance inOutDist = Units.Meters.of(-1);
		Distance leftRightDist = Units.Meters.of(0.5);//rightSide.getAsBoolean() ? Units.Meters.of(0.5) : Units.Meters.of(-0.5);
		Transform2d distAwayTransform = new Transform2d(inOutDist,leftRightDist, new Rotation2d());
		Pose2d foundReef = closest.transformBy(distAwayTransform);

		APTarget target = new APTarget(foundReef).withEntryAngle(foundReef.getRotation());

		return Commands.run(() ->{

				ChassisSpeeds robotRelativeSpeeds = DriveSubsystem.mInstance.getDrivetrain().getFieldVelocity();
  				Pose2d pose = DriveSubsystem.mInstance.getPose();

  				APResult output = kAutopilot.calculate(pose, robotRelativeSpeeds, target);

				
				DriveSubsystem.mInstance.getDrivetrain().setControl(m_request
				.withVelocityX(output.vx())
				.withVelocityY(output.vy())
				.withTargetDirection(output.targetAngle()));
			}
			)
    		//.until(/* snip */)
    		.finallyDo(DriveSubsystem.mInstance::stop);
		
	}

	public Command autoAlign(BooleanSupplier rightSide)
	{

		//find closest tag

		Branch closestBranch =  FieldLayout.Branch.getClosestBranch(DriveSubsystem.mInstance.getPose(), true);

		 
		Pose2d closest = FieldLayout.getCoralScoringPose( closestBranch);
		
		Distance inOutDist = Units.Meters.of(-1);
		Distance leftRightDist = rightSide.getAsBoolean() ? Units.Meters.of(0.5) : Units.Meters.of(-0.5);
		Transform2d distAwayTransform = new Transform2d(inOutDist,leftRightDist, new Rotation2d());

		
		return driveToPose.driveToPose(closest.transformBy(distAwayTransform));
		
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
