package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public class NomadMathUtil {

  public static double FIELD_WIDTH = 8.0137;
  public static double FIELD_LENGTH = 16.54175;

  public static Rotation2d getDirection(Transform2d transform) {
    return getDirection(transform.getTranslation());
  }

  public static Rotation2d getDirection(Translation2d transform) {
    // add tiny number so that 0/0 comes out to 0 angle, not a div by 0 error
    return new Rotation2d(transform.getX(), transform.getY());
  }

  public static Rotation2d getDirection(Pose2d tail, Pose2d head) {
    return getDirection(head.getTranslation().minus(tail.getTranslation()));
  }

  public static double getDistance(Transform2d transform) {
    return getDistance(transform.getTranslation());
  }

  public static double getDistance(Translation2d transform) {
    return transform.getNorm();
  }

  public static double subtractkS(double voltage, double kS) {
    if (Math.abs(voltage) <= kS) {
      voltage = 0;
    } else {
      voltage -= Math.copySign(kS, voltage);
    }
    return voltage;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle, double flipThreshold) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > flipThreshold) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public static PathPlannerTrajectory.State mirrorState(PathPlannerTrajectory.State blueState) {
    PathPlannerTrajectory.State newState = new PathPlannerTrajectory.State();
    newState.accelerationMpsSq = blueState.accelerationMpsSq;
    newState.curvatureRadPerMeter = -blueState.curvatureRadPerMeter;
    newState.headingAngularVelocityRps = -blueState.headingAngularVelocityRps;
    newState.targetHolonomicRotation =
        Rotation2d.fromRadians(Math.PI - blueState.targetHolonomicRotation.getRadians());
    newState.positionMeters =
        new Translation2d(
            FIELD_LENGTH - blueState.positionMeters.getX(), blueState.positionMeters.getY());
    newState.timeSeconds = blueState.timeSeconds;
    newState.velocityMps = newState.velocityMps;
    newState.constraints = blueState.constraints;
    return newState;
  }

  public static PathPlannerTrajectory.State mirrorState(
      PathPlannerTrajectory.State blueState, DriverStation.Alliance alliance) {
    if (alliance != Alliance.Red) {
      return blueState;
    }
    return mirrorState(blueState);
  }

  public static Pose2d mirrorPose(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH - bluePose.getX(),
        bluePose.getY(),
        Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians()));
  }

  public static Pose2d mirrorPose(Pose2d bluePose, DriverStation.Alliance alliance) {
    if (alliance != Alliance.Red) {
      return bluePose;
    }
    return mirrorPose(bluePose);
  }

  public static REVLibError retryRev(Supplier<REVLibError> command) {
    REVLibError error = REVLibError.kOk;
    for (int i = 0; i < 5; i++) {
      error = command.get();
      if (error == REVLibError.kOk) {
        return error;
      }
    }
    DriverStation.reportError(error.toString(), true);
    return error;
  }
}
