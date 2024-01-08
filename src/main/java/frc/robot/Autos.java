package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseS;

public class Autos {
  private DrivebaseS m_drivebaseS;

  public Autos(DrivebaseS drivebaseS) {
    m_drivebaseS = drivebaseS;
  }

  public Command choreo() {
    return m_drivebaseS.pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }
}
