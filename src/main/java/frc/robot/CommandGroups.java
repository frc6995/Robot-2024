package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.climber.ClimberS;
import frc.robot.subsystems.intake.IntakeRollerS;
import frc.robot.subsystems.intake.pivot.IntakePivotS;
import frc.robot.subsystems.shooter.midtake.MidtakeS;
import frc.robot.subsystems.shooter.pivot.ShooterPivotS;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsS;
import frc.robot.subsystems.vision.BlobDetectionCamera;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class CommandGroups {
  private DrivebaseS m_drivebaseS;
  private IntakePivotS m_intakePivotS;
  private IntakeRollerS m_intakeRollerS;
  private MidtakeS m_midtakeS;
  private ShooterPivotS m_shooterPivotS;
  private ShooterWheelsS m_shooterWheelsS;
  private ClimberS m_climberS;
  private BlobDetectionCamera m_noteCamera;
  private LightStripS m_lightStripS;

  public CommandGroups(
    DrivebaseS drivebaseS, 
    BlobDetectionCamera noteCamera,
    IntakePivotS intakePivotS,
    IntakeRollerS intakeRollerS,
    MidtakeS midtakeS,
    ShooterPivotS shooterPivotS,
    ShooterWheelsS shooterWheelsS,
    ClimberS climberS,
    LightStripS lightStripS) {
    m_drivebaseS = drivebaseS;
    m_intakePivotS = intakePivotS;
    m_intakeRollerS = intakeRollerS;
    m_midtakeS = midtakeS;
    m_shooterPivotS = shooterPivotS;
    m_shooterWheelsS = shooterWheelsS;
    m_climberS = climberS;
    m_lightStripS = lightStripS;
    m_noteCamera = noteCamera;
  }
  /**
   * This command deploys the intake, runs the intake rollers, and prepares to receive the 
   * note in the midtake.
   * 
   * End: When note is in midtake
   */
  public Command deployRunIntake() {
    return parallel(
      m_intakePivotS.deploy(),
      m_intakeRollerS.intakeC()
    );
  }
  public Command retractStopIntake() {
    return parallel(
      m_intakePivotS.retract(),
      m_intakeRollerS.stopC()
    );
  }
  public Command midtakeReceiveNote() {
    return m_midtakeS.intakeC().withTimeout(10); // TODO replace with sensor logic
  }
  public Command choreo() {
    return m_drivebaseS.pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }

  public Command driveToNote() {
    return m_drivebaseS.run(()->{
      if (m_noteCamera.hasTarget() &&
        Math.abs(m_noteCamera.getPitch() - 0.22) > 0.02 &&
        Math.abs(m_noteCamera.getYaw()) > 0.02
      ) {
        m_drivebaseS.drive(
          new ChassisSpeeds(
            MathUtil.clamp(6 * (0.22 - m_noteCamera.getPitch()), -1, 1),
            0,
            MathUtil.clamp(3* m_noteCamera.getYaw(), -1, 1)
          )
        );
      } else {
        m_drivebaseS.drive(new ChassisSpeeds());
      }
    });

  }
}
