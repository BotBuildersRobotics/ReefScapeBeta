package frc.robot.subsystems.vision;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightConfig;
import frc.robot.Robot;


public class LimelightConstants {
	public static final String kLimelightName = "limelight-back";

	public static final int kEnabledPipeline = 0;
	public static final int kDisabledPipeline = 1;
	public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);

	public static Pose3d kRobotToCameraOffset;

	static {
		
			kRobotToCameraOffset = new Pose3d(
					new Translation3d(Units.Centimeters.of(50), Units.Centimeters.of(15), Units.Centimeters.of(0)),
					new Rotation3d(Units.Degree.of(0), Units.Degree.of(-15), Units.Degree.of(0)));
		
	}

	public static final LimelightConfig getVisionIOConfig() {
		LimelightConfig config = new LimelightConfig();
		config.name = kLimelightName;
		config.robotToCameraOffset = kRobotToCameraOffset;
		return config;
	}

	public static final VisionIOLimelight getVisionIO() {
		
		return new VisionIOLimelight();
		
	}

	public static final int agreedHeadingUpdatesThreshold = 100;
	public static final Angle agreedHeadingUpdateEpsilon = Units.Degrees.of(2.0);
	public static final int agreedTranslationUpdatesThreshold = 100;
	public static final Distance agreedTranslationUpdateEpsilon = Units.Centimeters.of(10.0);
}