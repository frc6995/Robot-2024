package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.vision.BlobDetectionCamera;

public class Autos {
  private DrivebaseS m_drivebaseS;
  private BlobDetectionCamera m_noteCamera;

  public Autos(DrivebaseS drivebaseS, BlobDetectionCamera noteCamera) {
    m_drivebaseS = drivebaseS;
    m_noteCamera = noteCamera;
  }

  public Command choreo() {
    return m_drivebaseS.pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }

  public Command driveToNote() {
    return m_drivebaseS.run(()->{
      if (m_noteCamera.hasTarget()) {
        m_drivebaseS.drive(
          new ChassisSpeeds(
            MathUtil.clamp(6 * (0.22 - m_noteCamera.getPitch()), -1, 1),
            0,
            MathUtil.clamp(3* m_noteCamera.getYaw(), -1, 1)
          )
        );
      }
    });

  }
}
