package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.climber.ClimberS;
import frc.robot.subsystems.drive.DrivebaseS;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.intake.IntakeRollerS;
import frc.robot.subsystems.intake.pivot.IntakePivotS;
import frc.robot.subsystems.shooter.Interpolation;
import frc.robot.subsystems.shooter.feeder.ShooterFeederS;
import frc.robot.subsystems.shooter.midtake.MidtakeS;
import frc.robot.subsystems.shooter.pivot.ShooterPivotS;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsS;
import frc.robot.subsystems.vision.BlobDetectionCamera;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import monologue.Annotations.Log;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

public class CommandGroups {
  private DrivebaseS m_drivebaseS;
  private IntakePivotS m_intakePivotS;
  private IntakeRollerS m_intakeRollerS;
  private MidtakeS m_midtakeS;
  private ShooterFeederS m_shooterFeederS;
  private ShooterPivotS m_shooterPivotS;
  private ShooterWheelsS m_shooterWheelsS;
  // private ClimberS m_climberS;
  private BlobDetectionCamera m_noteCamera;
  private LightStripS m_lightStripS;

  public CommandGroups(
      DrivebaseS drivebaseS,
      BlobDetectionCamera noteCamera,
      IntakePivotS intakePivotS,
      IntakeRollerS intakeRollerS,
      MidtakeS midtakeS,
      ShooterFeederS shooterFeederS,
      ShooterPivotS shooterPivotS,
      ShooterWheelsS shooterWheelsS,
      // ClimberS climberS,
      LightStripS lightStripS) {
    m_drivebaseS = drivebaseS;
    m_intakePivotS = intakePivotS;
    m_intakeRollerS = intakeRollerS;
    m_midtakeS = midtakeS;
    m_shooterFeederS = shooterFeederS;
    m_shooterPivotS = shooterPivotS;
    m_shooterWheelsS = shooterWheelsS;
    // m_climberS = climberS;
    m_lightStripS = lightStripS;
    m_noteCamera = noteCamera;
  }

  /**
   * This command deploys the intake, runs the intake rollers, and prepares to
   * receive the
   * note in the midtake.
   * 
   * End: When note is in midtake
   */
  public Command deployRunIntake(Trigger overrideTOF) {
    Trigger hasNote = m_midtakeS.hasNote.and(overrideTOF.negate());
    return parallel(
        sequence(
            parallel(
                m_intakePivotS.deploy(),
                waitSeconds(0.2).andThen(m_intakeRollerS.intakeC()),
                waitSeconds(0.1).andThen(
                    // m_midtakeS.intakeC()
                    m_midtakeS.runVoltage(() -> MidtakeS.Constants.IN_VOLTAGE, () -> MidtakeS.Constants.IN_VOLTAGE)))
                .until(hasNote),

            parallel(
                sequence(
                    m_intakePivotS.deploy()),
                m_intakeRollerS.slowInC(),
                m_midtakeS.runVoltage(() -> 1, () -> 1),
                m_shooterFeederS.runVoltageC(() -> 1))
                .withTimeout(1)
                .until(hasNote.negate())
                .onlyIf(hasNote),
            // intentionally interrupt the current command to fragment the group
            parallel(
                new ScheduleCommand(m_intakeRollerS.slowInC().withTimeout(1).andThen(m_intakeRollerS.stopC())),
                new ScheduleCommand(m_intakePivotS.hold().withTimeout(0.3).andThen(m_intakePivotS.retract())),
                new ScheduleCommand(
                    parallel(
                        m_midtakeS.runVoltage(() -> 0, () -> -1),
                        m_shooterFeederS.backupC())

                        .withTimeout(3)
                        .until(hasNote)
                        .andThen(parallel(
                            m_midtakeS.stopOnceC(), m_shooterFeederS.stopC())))))

    )
    // .or(m_midtakeS.recvNote.and(m_midtakeS.isRunning)))
    ;
    // .andThen(
    // parallel(
    // sequence(
    // m_midtakeS.stopC(),
    // waitSeconds(0.5),
    // m_midtakeS.outtakeC().withTimeout(0.3),
    // m_midtakeS.stopC()),
    // waitSeconds(0.5).andThen(retractStopIntake())
    // )
    // );
    // .andThen(
    // parallel(
    // m_intakeRollerS.slowInC(),
    // m_midtakeS.outtakeC()
    // ).withTimeout(0.3)
    // );
  }

  public Command retractStopIntake() {
    return parallel(
        m_intakePivotS.retract(),
        m_intakeRollerS.stopC());
  }

  // public Command midtakeReceiveNote() {
  // return m_midtakeS.intakeC().withTimeout(10); // TODO replace with sensor
  // logic
  // }
  public Command choreo() {
    return m_drivebaseS.pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }

  public Command autoPickupC() {
    return defer(() -> {
      var note = m_noteCamera.getBestTarget(m_drivebaseS.getPose());
      if (note.isEmpty()) {
        return Commands.none();
      } // TODO driver feedback? Must be proxied for duration
      var pickup = note.get().transformBy(new Transform2d(new Translation2d(-1, 0), new Rotation2d()));
      return parallel(
          sequence(
              m_drivebaseS.chasePoseC(() -> pickup),
              m_drivebaseS.run(() -> {
                m_drivebaseS.drive(new ChassisSpeeds(0.5, 0, 0));
              })),
          deployRunIntake(new Trigger(() -> false)));
    }, Set.of(m_drivebaseS, m_intakePivotS, m_intakeRollerS));
  }

  public Command faceNoteC(InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleConsumer rumble) {
    return sequence(
        run(() -> rumble.accept(0.3))
            .finallyDo(() -> rumble.accept(0))
            .until(() -> m_noteCamera.getBestTarget(m_drivebaseS.getPose()).isPresent()),
        m_drivebaseS.defer(() -> {
          var note = m_noteCamera.getBestTarget(m_drivebaseS.getPose());
          if (note.isEmpty()) {
            return new ScheduleCommand(
                run(() -> rumble.accept(0.5))
                    .withTimeout(0.5)
                    .finallyDo(() -> rumble.accept(0)));
          } // TODO driver feedback? Must be proxied for duration
          return parallel(
              sequence(
                  m_drivebaseS.manualFieldHeadingDriveC(fwdXAxis, fwdYAxis,
                      () -> Pathing.speakerDirection(
                          m_drivebaseS.getPose(),
                          note.get().getTranslation()).getRadians() + Math.PI,
                      () -> 0)
          // m_drivebaseS.chasePoseC(()->pickup),
          // m_drivebaseS.run(()->{
          // m_drivebaseS.drive(new ChassisSpeeds(0.5, 0, 0));
          // })
          ));
        }));
  }

  public Command driveToNote() {
    return m_drivebaseS.run(() -> {
      if (m_noteCamera.hasTarget() &&
          Math.abs(m_noteCamera.getPitch() - 0.22) > 0.02 &&
          Math.abs(m_noteCamera.getYaw()) > 0.02) {
        m_drivebaseS.drive(
            new ChassisSpeeds(
                MathUtil.clamp(6 * (0.22 - m_noteCamera.getPitch()), -1, 1),
                0,
                MathUtil.clamp(3 * m_noteCamera.getYaw(), -1, 1)));
      } else {
        m_drivebaseS.drive(new ChassisSpeeds());
      }
    });
  }

  public Translation2d speaker() {
    return NomadMathUtil.mirrorTranslation(
        Constants.Poses.SPEAKER,
        AllianceWrapper.getAlliance());
  }

  public double directionToSpeaker() {
    return Pathing.speakerDirection(
        m_drivebaseS.getPose(),
        speaker()).getRadians();
  }

  public double distanceToSpeaker() {
    return Pathing.speakerDistance(
        m_drivebaseS.getPose(),
        speaker());
  }

  public double pivotAngle() {
    return Interpolation.PIVOT_MAP.get(distanceToSpeaker());
  }

  public Command feed() {
    return parallel(
        m_midtakeS.intakeC(),
        m_shooterFeederS.feedC());
  }

  public Command centerWingNote(PathPlannerPath startToPrePickup) {
    var path = startToPrePickup.getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return parallel(
        m_intakePivotS.deploy().asProxy(),
        m_shooterPivotS.rotateToAngle(() -> 2.367).asProxy(),

        m_shooterWheelsS.spinC(() -> 6000, () -> 6000),
        sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            m_drivebaseS.pathPlannerCommand(startToPrePickup),
            waitSeconds(1),
            feed().asProxy().withTimeout(1),
            parallel(
                m_intakeRollerS.intakeC().asProxy(),
                feed().asProxy(),
                m_drivebaseS.run(
                    () -> m_drivebaseS.drive(new ChassisSpeeds(0.2, 0, 0))).withTimeout(4).until(m_midtakeS.hasNote)
                    .andThen(m_drivebaseS.stopOnceC())

            ))

    );
  }
  public Command centerFourWingNote() {
    var path = PathPlannerPath.fromChoreoTrajectory("W2.1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return deadline(
      sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            m_drivebaseS.choreoCommand("W2.1"),
            m_drivebaseS.stopOnceC(),
            feed().asProxy().withTimeout(1),
            deadline(
                sequence(
                  m_drivebaseS.choreoCommand("W2.2"),
                  m_drivebaseS.stopOnceC(),
                  waitSeconds(1),
                  m_drivebaseS.choreoCommand("W2.3"),
                  m_drivebaseS.stopOnceC(),
                  waitSeconds(1),
                  m_drivebaseS.choreoCommand("W2.4"),
                  m_drivebaseS.stopOnceC(),
                  waitSeconds(2)
                ),
                m_intakeRollerS.intakeC().asProxy(),
                feed().asProxy()
            )),

        m_intakePivotS.deploy().asProxy(),
        m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),

        m_shooterWheelsS.spinC(() -> 6000, () -> 6000).asProxy()


    );
  }

  public Command centerFourWingMidline() {
    return sequence(
      centerFourWingNote(),
      autoIntakeCycle("W2.5"),
      m_drivebaseS.stopOnceC(),
      feed().asProxy().withTimeout(5)
    );
  }

  public Command w3w2() {
    return parallel(
        m_intakePivotS.deploy().asProxy(),
        m_shooterPivotS.rotateToAngle(() -> pivotAngle()).asProxy(),

        m_shooterWheelsS.spinC(() -> 6000, () -> 6000),
        sequence(

            m_drivebaseS.pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory("W3.1")),
            feed().asProxy().withTimeout(1),
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(
                        PathPlannerPath.fromChoreoTrajectory("W3.2")),
                    waitSeconds(1),
                    m_drivebaseS.pathPlannerCommand(
                        PathPlannerPath.fromChoreoTrajectory("W3.3")),
                    m_drivebaseS.stopC()),
                m_intakeRollerS.intakeC().asProxy(),
                feed().asProxy()

            )

        ))

    ;
  }

  public Command autoIntakeCycle(String choreoTrajectory) {
    return deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(
                      PathPlannerPath.fromChoreoTrajectory(choreoTrajectory)),
                    m_drivebaseS.stopOnceC()

                ),

                deployRunIntake(new Trigger(() -> false)).asProxy());
  }
  public Command c5() {
    var path = PathPlannerPath.fromChoreoTrajectory("C5.1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return parallel(
        new ScheduleCommand(m_intakePivotS.deploy().asProxy()),
        m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
        m_shooterWheelsS.spinC(() -> 6000, () -> 6000).asProxy(),
        sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            m_drivebaseS.choreoCommand("C5.1"),
            m_drivebaseS.stopOnceC(),
            feed().asProxy().withTimeout(1),
            autoIntakeCycle("C5.2"),
            feed().asProxy().withTimeout(1),
            autoIntakeCycle("C5.3"),
            feed().asProxy().withTimeout(0.5)
        ));
  }
}
