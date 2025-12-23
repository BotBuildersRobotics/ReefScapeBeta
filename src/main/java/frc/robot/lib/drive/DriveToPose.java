package frc.robot.lib.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.units.CurrentUnit;



public class DriveToPose extends Command{
    private CommandSwerveDrivetrain m_swerveSubsystem;

    private Pose2d m_targetPose = new Pose2d();

    private static SwerveRequest.ApplyRobotSpeeds m_swerveRequest = new ApplyRobotSpeeds();


    public DriveToPose(DriveSubsystem swerveSubsystem) {
       
        m_swerveSubsystem = swerveSubsystem.getDrivetrain();
    
        addRequirements(m_swerveSubsystem);
    
    }

    /**
   * Method to generate a command to follow a path to a waypoint | 
   * Auto adjusts to the waypoint upon arrival | 
   * Auto adjusts override if within 0.25 meters of waypoint | 
   * Moves elevator to desired reef level if within 1 meter of waypoint | 
   * @param waypoint The target waypoint to align to
   * @return Command to follow the path to the waypoint
   */
  private Command getPathFromWaypoint(Pose2d waypoint) {
    // Create waypoints for pathplanner path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(m_swerveSubsystem.getState().Pose.getTranslation(), getPathVelocityHeading(m_swerveSubsystem.getFieldVelocity(), waypoint)),
      waypoint
    );
    
    // Create path constraints
    PathConstraints pathConstraints = new PathConstraints(3, 3, 180, 360);

    // Create pathplanner path
    PathPlannerPath path = new PathPlannerPath(
                                              waypoints, 
                                              pathConstraints, 
                                              new IdealStartingState(getVelocityMagnitude(m_swerveSubsystem.getFieldVelocity()), m_swerveSubsystem.getState().Pose.getRotation()),
                                              new GoalEndState(0.0, waypoint.getRotation()));
    path.preventFlipping = true;
  
    m_targetPose = waypoint;

    return AutoBuilder.followPath(path);
  }

  /**
   * Method to get the velocity magnitude from chassis speeds 
   * @param cs ChassisSpeeds of the robot
   * @return LinearVelocity magnitude of the robot 
   */
  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  
  }

  /**
   * Method to get the heading based on path velocity or target pose
   * @param cs ChassisSpeeds of the robot
   * @param targetPose Target Pose2d to align to
   * @return Rotation2d heading for the path 
   */
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d targetPose){
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) { // If the robot is moving slower than 0.25 m/s, face the target
      var diff = targetPose.minus(m_swerveSubsystem.getState().Pose).getTranslation();
      return (diff.getNorm() < 0.01) ? targetPose.getRotation() : diff.getAngle(); // If the robot is within 1 cm of the target, keep the target rotation
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  public Command driveToPose(Pose2d targetPose){
    return Commands.defer(()-> {
      return getPathFromWaypoint(targetPose);
    }, Set.of()); 
  }


    
}
