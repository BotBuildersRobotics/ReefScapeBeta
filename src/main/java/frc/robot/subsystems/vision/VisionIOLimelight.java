
package frc.robot.subsystems.vision;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightConfig;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightConstants;

public class VisionIOLimelight extends VisionIO {
	private Pose2d latestEstimate = new Pose2d();
	private Time latestEstimateTime = Units.Seconds.of(0.0);
	private LimelightConfig config = new LimelightConfig();
	protected StructPublisher<Pose2d> visPose = NetworkTableInstance.getDefault()
			.getTable("SmartDashboard/Vision")
			.getStructTopic("", Pose2d.struct)
			.publish();

	@Override
	public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {
		SmartDashboard.putNumber(config.name + "/Tag Count", poseEstimate.tagCount);
		SmartDashboard.putNumber(config.name + "/FGPA Timestamp", Timer.getFPGATimestamp());
		SmartDashboard.putNumber(
				config.name + "/Estimate to FGPA Timestamp", Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));
		if (poseEstimate.tagCount >= minTagNum) {
			latestEstimate = poseEstimate.pose;
			latestEstimateTime = Units.Seconds.of(poseEstimate.timestampSeconds);
			visPose.set(poseEstimate.pose);
			DriveSubsystem.mInstance.getGeneratedDrive();
			DriveSubsystem.mInstance.addVisionUpdate(
					poseEstimate.pose,
					Units.Seconds.of(poseEstimate.timestampSeconds),
					LimelightConstants.enabledVisionStdDevs.times(poseEstimate.avgTagDist));
		}
	}

	public Pose2d getLatestEstimate() {
		return latestEstimate;
	}

	public Time getLatestEstimateTime() {
		return latestEstimateTime;
	}

	@Override
	public void update() {
		//updateGyro();	
		//if (DriverStation.getAlliance().get() == Alliance.Blue) {
			setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name), 1);
		//} else if (DriverStation.getAlliance().get() == Alliance.Red) {
		//	setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiRed(config.name), 1);
		//}
		//setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name), 1);
		//setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name), 1);

		SmartDashboard.putBoolean(config.name + "/Disabled", disabled);
	}

	@Override
	public void disable(boolean disable) {
		super.disable(disable);

		if (disabled) {
			LimelightHelpers.setPipelineIndex(config.name, LimelightConstants.kDisabledPipeline);
		} else {
			LimelightHelpers.setPipelineIndex(config.name, LimelightConstants.kEnabledPipeline);
		}
	}

	private void updateGyro() {

		Rotation2d theta = DriveSubsystem.mInstance.getPose().getRotation();
		LimelightHelpers.SetRobotOrientation(config.name, theta.getDegrees(), 0, 0, 0, 0, 0);
	}

	public void updateConfig(LimelightConfig config) {
		this.config = config;
	}
}