package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.AllianceWrapper;

public class Pathing {

  public static double aimingFFVelocity(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
    return speakerRelativeSpeeds(currentPose, fieldRelativeSpeeds, 
      targetTranslation
    ).vyMetersPerSecond
      / speakerDistance(currentPose, targetTranslation);
  }
  public static double velocityTowards(
    Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
       return speakerRelativeSpeeds(currentPose, fieldRelativeSpeeds, 
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
  public static final Pose2d BLUE_AMP = new Pose2d(1.759, 7.786, new Rotation2d(-Math.PI/2));
  public static final Pose2d RED_AMP = new Pose2d(14.665, 7.786, new Rotation2d(-Math.PI/2));
  public static Pose2d getOwnAmp() {
    if (AllianceWrapper.isRed()) {
      return RED_AMP;
    } else {
      return BLUE_AMP;
    }
  }

  public static final Pose2d BLUE_PREAMP = new Pose2d(1.759, 7.3, new Rotation2d(-Math.PI/2));
  public static final Pose2d RED_PREAMP = new Pose2d(14.665, 7.3, new Rotation2d(-Math.PI/2));
  public static Pose2d getOwnPreAmp() {
    if (AllianceWrapper.isRed()) {
      return RED_PREAMP;
    } else {
      return BLUE_PREAMP;
    }
  }

}
