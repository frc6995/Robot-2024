package frc.robot.subsystems.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import static frc.robot.Constants.DriveConstants.*;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Pathing {
 public static final PPHolonomicDriveController m_holonomicDriveController =
      new PPHolonomicDriveController(
          new PIDConstants(4, 0, 0.001),
          new PIDConstants(4, 0, 0.001),
          0.02,
          MAX_MODULE_SPEED_MPS,
          ModuleConstants.FL.centerOffset.getNorm());
  public static final HolonomicPathFollowerConfig m_pathPlannerConfig =
  
      new HolonomicPathFollowerConfig(
          new PIDConstants(4, 0, 0.001),
          new PIDConstants(6, 0, 0.001),
          MAX_MODULE_SPEED_MPS,
          ModuleConstants.FL.centerOffset.getNorm(),
          new ReplanningConfig(false, false, 0.1, 0.1));
    /**
   * For use with PPChasePoseCommand Generates a PathPlannerTrajectory on the fly to drive to the
   * target pose. Takes into account the current speed of the robot for the start point. The
   * returned PathPlannerTrajectory will go straight towards the target from the robot pose. The
   * component of the current velocity that points toward the target will be used as the initial
   * velocity of the trajectory.
   *
   * @param robotPose the current robot pose
   * @param target the target pose
   * @param currentSpeeds a Translation2d where x and y are the robot's x and y field-relative
   *     speeds in m/s.
   * @return a PathPlannerTrajectory to the target pose.
   */
  public static PathPlannerTrajectory generateTrajectoryToPose(
      Pose2d robotPose, Pose2d target, ChassisSpeeds currentSpeeds, PathConstraints constraints) {
    Translation2d robotToTargetTranslation =
        target.getTranslation().minus(robotPose.getTranslation());
    PathPlannerPath pathPlannerTrajectory =
        new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(
                List.of(
                    new Pose2d(robotPose.getTranslation(), robotToTargetTranslation.getAngle()),
                    new Pose2d(target.getTranslation(), robotToTargetTranslation.getAngle()))),
            List.of(
                new RotationTarget(0, robotPose.getRotation()),
                new RotationTarget(1, target.getRotation(), false)),
            List.of(),
            List.of(),
            // Start point. At the position of the robot, initial travel direction toward
            // the target,
            // robot rotation as the holonomic rotation, and putting in the (possibly 0)
            // velocity override.
            constraints,
            new GoalEndState(0, target.getRotation()),
            false);
    return new PathPlannerTrajectory(pathPlannerTrajectory, currentSpeeds, robotPose.getRotation());
  }

  public static List<Pose2d> ppTrajectoryToPoseList(PathPlannerTrajectory traj) {
    return traj.getStates().stream()
    .map(
        (Function<State, Pose2d>)
            (State state) -> {
              return new Pose2d(
                  state.positionMeters, state.targetHolonomicRotation);
            })
    .collect(Collectors.toList());
  }

  public static double aimingFFVelocity(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
    return speakerRelativeSpeeds(currentPose, fieldRelativeSpeeds, 
      targetTranslation
    ).vyMetersPerSecond
      / speakerDistance(currentPose, targetTranslation);
  }
  public static double velocityTorwardsSpeaker(
    Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
       return -speakerRelativeSpeeds(currentPose, fieldRelativeSpeeds, 
      targetTranslation
    ).vxMetersPerSecond;
    }
  
  /**
   * Forward is AWAY from the speaker
   * @param currentPose
   * @param fieldRelativeSpeeds
   * @param targetTranslation
   * @return
   */
  public static ChassisSpeeds speakerRelativeSpeeds(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, 
      speakerDirection(currentPose, targetTranslation)
    );
  }
  public static Rotation2d speakerDirection(Pose2d currentPose, Translation2d target) {
    return target.minus(currentPose.getTranslation()).getAngle().minus(new Rotation2d(Math.PI));
  }
  public static double speakerDistance(Pose2d currentPose, Translation2d target) {
    return currentPose.getTranslation().minus(target).getNorm();
  }
  public static Pose2d timeAdjustedPose(Pose2d currentPose, ChassisSpeeds robotRelativeSpeeds, double time) {
    return currentPose.transformBy(new Transform2d(
      robotRelativeSpeeds.vxMetersPerSecond * time,
      robotRelativeSpeeds.vyMetersPerSecond*time,
      new Rotation2d(robotRelativeSpeeds.omegaRadiansPerSecond * time)
    ));
  }
  public static final Pose2d BLUE_AMP = new Pose2d(1.818, 7.67, new Rotation2d(-Math.PI/2));

}
