package frc.robot;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.amp.AmpRollerS;
import frc.robot.subsystems.amp.pivot.AmpPivotS;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.drive.Swerve;
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
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

import java.util.function.DoubleConsumer;

import static frc.robot.util.Defaults.*;
public class CommandGroups {
  private Swerve m_drivebaseS;
  private IntakePivotS m_intakePivotS;
  private IntakeRollerS m_intakeRollerS;
  private MidtakeS m_midtakeS;
  private AmpPivotS m_ampPivotS;
  private AmpRollerS m_ampRollerS;
  private ShooterFeederS m_shooterFeederS;
  private ShooterPivotS m_shooterPivotS;
  private ShooterWheelsS m_shooterWheelsS;
  // private ClimberS m_climberS;
  private BlobDetectionCamera m_noteCamera;
  private LightStripS m_lightStripS;
  private CommandXboxController m_driverController;

  public AutoFactory m_autoFactory;

  public CommandGroups(
      Swerve drivebaseS,
      BlobDetectionCamera noteCamera,
      IntakePivotS intakePivotS,
      IntakeRollerS intakeRollerS,
      MidtakeS midtakeS,
      AmpPivotS ampPivotS,
      AmpRollerS ampRollerS,
      ShooterFeederS shooterFeederS,
      ShooterPivotS shooterPivotS,
      ShooterWheelsS shooterWheelsS,
      // ClimberS climberS,
      LightStripS lightStripS,
      CommandXboxController driverController) {
    m_drivebaseS = drivebaseS;
    m_intakePivotS = intakePivotS;
    m_intakeRollerS = intakeRollerS;
    m_midtakeS = midtakeS;
    m_shooterFeederS = shooterFeederS;
    m_shooterPivotS = shooterPivotS;
    m_shooterWheelsS = shooterWheelsS;
    m_ampPivotS = ampPivotS;
    m_ampRollerS = ampRollerS;
    m_lightStripS = lightStripS;
    m_noteCamera = noteCamera;
    m_driverController = driverController;
    m_autoFactory = Choreo.createAutoFactory(
      m_drivebaseS,
      m_drivebaseS::getPose,
      m_drivebaseS::choreoController,
      AllianceWrapper::isRed,
      new AutoFactory.AutoBindings());
  }



  /**
   * This command deploys the intake, runs the intake rollers, and prepares to
   * receive the
   * note in the midtake.
   * 
   * End: When note is in midtake
   */
  public Command deployRunIntake(Trigger overrideTOF) {
    Trigger hasNote = m_midtakeS.hasNote;
    return parallel(
        sequence(
            parallel(
                m_shooterFeederS.runVoltageC(() -> 0),
                m_intakePivotS.deploy(),
                waitSeconds(0.2).andThen(m_intakeRollerS.intakeC()),
                waitSeconds(0.1).andThen(
                    // m_midtakeS.intakeC()
                    m_midtakeS.runVoltage(() -> MidtakeS.Constants.IN_VOLTAGE, () -> MidtakeS.Constants.IN_VOLTAGE)))
                .until(m_midtakeS.hasNote.and(overrideTOF.negate())),

            parallel(
                new ScheduleCommand(rumbleDriver(0.7).withTimeout(0.75)),
                new ScheduleCommand(m_lightStripS.stateC(() -> States.IntakedNote).withTimeout(0.75)
                  .andThen(m_lightStripS.stateC(()->States.HasNote).withTimeout(1.5))),
                sequence(
                    waitSeconds(0.5),
                    m_intakePivotS.retract()),
                m_intakeRollerS.slowInC(),
                m_midtakeS.runVoltage(() -> 6, () -> 0),
                m_shooterFeederS.runVoltageC(() -> 0))
                .withTimeout(1)
                .until(hasNote.negate())
                .onlyIf(hasNote),
            // intentionally interrupt the current command to fragment the group
            parallel(
                new ScheduleCommand(m_intakeRollerS.slowInC().withTimeout(1).andThen(m_intakeRollerS.stopC())),
                new ScheduleCommand(m_intakePivotS.retract()),
                new ScheduleCommand(
                    parallel(
                        m_midtakeS.runVoltage(() -> -1, () -> -1),
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

  // public Command autoPickupC() {
  //   return defer(() -> {
  //     var note = m_noteCamera.getBestTarget(m_drivebaseS.getPose());
  //     if (note.isEmpty()) {
  //       return Commands.none();
  //     } // TODO driver feedback? Must be proxied for duration
  //     var pickup = note.get().transformBy(new Transform2d(new Translation2d(-1, 0), ZERO_ROTATION2D));
  //     return parallel(
  //         sequence(
  //             m_drivebaseS.chasePoseC(() -> pickup),
  //             m_drivebaseS.run(() -> {
  //               m_drivebaseS.drive(new ChassisSpeeds(0.5, 0, 0));
  //             })),
  //         deployRunIntake(new Trigger(() -> false)));
  //   }, Set.of(m_drivebaseS, m_intakePivotS, m_intakeRollerS));
  // }

  public Command rumbleDriver(double speed) {
    return run(
        () -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, speed))
        .finallyDo(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
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
        m_drivebaseS.drive(ZERO_CHASSISSPEEDS);
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

  /** AUTOS */

  public Command s2Toc2() {

    var loop = m_autoFactory.newLoop("S2-C2");
    var trajectory = m_autoFactory.trajectory("S2-C2", loop);
    loop.enabled().onTrue(m_drivebaseS.resetPoseToBeginningC(trajectory));
    loop.enabled().onTrue(trajectory.cmd());
    return loop.cmd();
  }

  public Command fourNote() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S2-SH2", loop);
    var second = m_autoFactory.trajectory("SH2-C2", loop);
    var third = m_autoFactory.trajectory("C2-C1", loop);
    var fourth = m_autoFactory.trajectory("C1-C3", loop);
    loop.enabled()
    .onTrue(m_drivebaseS.resetPoseToBeginningC(first))
    .onTrue(m_intakePivotS.deploy())
    .whileTrue(spinDistance(this::distanceToSpeaker)).
    onTrue(first.cmd());

    second.atTime(0).onTrue(m_intakeRollerS.intakeC()).onTrue(feed());
    first.done()
      .onTrue(sequence(
        feed().withTimeout(0.75),
        new ScheduleCommand(second.cmd())
      ));
    second.done().onTrue(third.cmd());
    third.done().onTrue(fourth.cmd());
    fourth.done()
      .onTrue(m_drivebaseS.stopOnceC())
      .onTrue(m_intakeRollerS.stopOnceC())
      
      .onTrue(m_shooterWheelsS.stopC().withTimeout(0.0));
    
    return loop.cmd();
  }

  // public Command centerWingNote(PathPlannerPath startToPrePickup) {
  //   var path = startToPrePickup.getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return parallel(
  //       m_intakePivotS.deploy().asProxy(),
  //       m_shooterPivotS.rotateToAngle(() -> 2.367).asProxy(),

  //       m_shooterWheelsS.spinC(() -> 6000, () -> 6000),
  //       sequence(
  //           m_drivebaseS.resetPoseToBeginningC(path),
  //           m_drivebaseS.pathPlannerCommand(startToPrePickup),
  //           waitSeconds(1),
  //           feed().asProxy().withTimeout(1),
  //           parallel(
  //               m_intakeRollerS.intakeC().asProxy(),
  //               feed().asProxy(),
  //               m_drivebaseS.run(
  //                   () -> m_drivebaseS.drive(new ChassisSpeeds(0.2, 0, 0))).withTimeout(4).until(m_midtakeS.hasNote)
  //                   .andThen(m_drivebaseS.stopOnceC())

  //           ))

  //   );
  // }

  // public Command w1() {
  //   var path = PathPlannerPath.fromChoreoTrajectory("W1").getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return deadline(
  //     sequence(
  //           m_drivebaseS.resetPoseToBeginningC(path),
  //           Pathing.setRotationOverride(()->Optional.of(new Rotation2d(this.directionToSpeaker()))),
  //           m_drivebaseS.choreoCommand("W1"),
  //           m_drivebaseS.stopOnceC(),
  //           waitSeconds(1),
  //           feed().asProxy().withTimeout(5)
  //     ),
  //     m_shooterPivotS.rotateWithVelocity(
  //           this::pivotAngle,
  //           () -> Interpolation.dThetadX(distanceToSpeaker()) *
  //               -Pathing.velocityTorwardsSpeaker(
  //                   m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
  //                   speaker()))
  //           .asProxy(),

  //       spinDistance(this::distanceToSpeaker).asProxy()

  //   ).finallyDo(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
  // }
  // public Command centerFourWingNote(double endWait) {
  //   var path = PathPlannerPath.fromChoreoTrajectory("W2.1").getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return deadline(
  //       sequence(
  //           m_drivebaseS.resetPoseToBeginningC(path),
  //           Pathing.setRotationOverride(()->Optional.of(new Rotation2d(this.directionToSpeaker()))),
  //           m_drivebaseS.choreoCommand("W2.1"),
  //           m_drivebaseS.stopOnceC(),
  //           feed().asProxy().withTimeout(0.75),
  //           deadline(
  //               sequence(
  //                   m_drivebaseS.choreoCommand("W2.2"),
  //                   m_drivebaseS.stopOnceC(),
  //                   waitSeconds(0),
  //                   m_drivebaseS.choreoCommand("W2.3"),
  //                   m_drivebaseS.stopOnceC(),
  //                   waitSeconds(0),
  //                   m_drivebaseS.choreoCommand("W2.4"),
  //                   m_drivebaseS.stopOnceC(),
  //                   Pathing.clearRotationOverride(),
  //                   waitSeconds(endWait)),
  //               m_intakeRollerS.intakeC().asProxy(),
  //               feed().asProxy()
  //           )
  //       ),
  //       m_intakePivotS.deploy().asProxy(),
  //       m_shooterPivotS.rotateWithVelocity(
  //           this::pivotAngle,
  //           () -> Interpolation.dThetadX(distanceToSpeaker()) *
  //               -Pathing.velocityTorwardsSpeaker(
  //                   m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
  //                   speaker()))
  //           .asProxy(),

  //       spinDistance(this::distanceToSpeaker).asProxy()

  //   ).finallyDo(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
  // }

  public Command spinDistance(DoubleSupplier distance) {
    return m_shooterWheelsS.spinC(() -> Interpolation.LEFT_MAP.get(distance.getAsDouble()),
        () -> Interpolation.RIGHT_MAP.get(distance.getAsDouble()));
  }

  // public Command centerFourWingMidline() {
  //   return sequence(
  //       centerFourWingNote(0.25),
  //       parallel(
  //           sequence(
  //               autoIntakeCycle("W2.5", 1, true, 0.75, this::notAtMidline),
  //               m_drivebaseS.stopOnceC(),
  //               feed().asProxy().withTimeout(0.25),
  //               autoIntakeCycle("W2.6", 0.25, true, 0.75, this::notAtMidline),
  //               m_drivebaseS.stopOnceC(),
  //               feed().asProxy().withTimeout(5)
  //           ),
  //           m_shooterPivotS.rotateWithVelocity(
  //               this::pivotAngle,
  //               () -> Interpolation.dThetadX(distanceToSpeaker()) *
  //                   -Pathing.velocityTorwardsSpeaker(
  //                       m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
  //                       speaker()))
  //               .asProxy(),
  //           spinDistance(this::distanceToSpeaker).asProxy()));
  // }
  // public Command centerFourWingC3C4() {
  //   return sequence(
  //       centerFourWingNote(0.25),
  //       parallel(
  //           sequence(
  //               autoIntakeCycle("4Close-C3.1", 0.5, true, 0.75, this::notAtMidline),
  //               m_drivebaseS.stopOnceC(),
  //               feed().asProxy().withTimeout(0.25),
  //               autoIntakeCycle("4Close-C3.2", 0.25, true, 0.75, this::notAtMidline),
  //               m_drivebaseS.stopOnceC(),
  //               feed().asProxy().withTimeout(5)
  //           ),
  //           m_shooterPivotS.rotateWithVelocity(
  //               this::pivotAngle,
  //               () -> Interpolation.dThetadX(distanceToSpeaker()) *
  //                   -Pathing.velocityTorwardsSpeaker(
  //                       m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
  //                       speaker()))
  //               .asProxy(),
  //           spinDistance(this::distanceToSpeaker).asProxy()));
  // }


  // public Command w3w2() {
  //   return parallel(
  //       m_intakePivotS.deploy().asProxy(),
  //       m_shooterPivotS.rotateToAngle(() -> pivotAngle()).asProxy(),

  //       m_shooterWheelsS.spinC(() -> 6000, () -> 6000),
  //       sequence(

  //           m_drivebaseS.pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory("W3.1")),
  //           feed().asProxy().withTimeout(1),
  //           deadline(
  //               sequence(
  //                   m_drivebaseS.pathPlannerCommand(
  //                       PathPlannerPath.fromChoreoTrajectory("W3.2")),
  //                   waitSeconds(1),
  //                   m_drivebaseS.pathPlannerCommand(
  //                       PathPlannerPath.fromChoreoTrajectory("W3.3")),
  //                   m_drivebaseS.stopC()),
  //               m_intakeRollerS.intakeC().asProxy(),
  //               feed().asProxy()

  //           )

  //       ))

  //   ;
  // }

  // public Command autoIntakeCycle(String choreoTrajectory, double intakeTimeout, boolean feed, double aimTime, BooleanSupplier noNoteYet) {
  //   var path = PathPlannerPath.fromChoreoTrajectory(choreoTrajectory);
  //   var traj = path.getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   var startAimTime = traj.getTotalTimeSeconds() - aimTime;
  //   return deadline(

  //       sequence(
  //           m_drivebaseS.pathPlannerCommand(path),
  //           m_drivebaseS.stopOnceC()

  //       ),
  //       (aimTime > 0 ? waitSeconds(startAimTime).andThen(
  //           Pathing.setRotationOverride(
  //               () -> Optional.of(new Rotation2d(this.directionToSpeaker()))))
  //           : none()),
  //       (feed ? feed().asProxy().withTimeout(intakeTimeout) : waitSeconds(intakeTimeout)).andThen(
  //           deployRunIntake(new Trigger(noNoteYet)).asProxy()))
  //       .finallyDo(
  //           () -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
  // }

  public boolean notAtMidline() {
    return Math.abs(m_drivebaseS.getPose().getX() - 8.22) > 0.5;
  }
  // public Command c5() {
  //   var path = PathPlannerPath.fromChoreoTrajectory("C5S.1").getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return parallel(
  //       new ScheduleCommand(m_intakePivotS.deploy().asProxy()),
  //       m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
  //       spinDistance(this::distanceToSpeaker).asProxy(),
  //       sequence(
  //           m_drivebaseS.resetPoseToBeginningC(path),
  //           autoIntakeCycle("C5S.1", 3, false, 1, ()->false),
  //           feed().asProxy().withTimeout(0.5),
  //           autoIntakeCycle("C5S.2", 1, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.5),
  //           autoIntakeCycle("C5S.3", 0.2, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.5)));
  // }

  // public Command c5ThruStageBlue() {
  //   var path = PathPlannerPath.fromChoreoTrajectory("C5.1").getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return parallel(
  //       m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
  //       spinDistance(this::distanceToSpeaker).asProxy(),
  //       sequence(
  //           m_drivebaseS.resetPoseToBeginningC(path),
  //           // too-long intake delay
  //           autoIntakeCycle("C5.1", 2, false, 1, ()->false),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5.2", 1, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5.3", 0.2, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5.4", 0.2, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.3)));
  // }

  // public Command c4c3() {
  //   var path = PathPlannerPath.fromChoreoTrajectory("C5 (1).1").getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return parallel(
  //       m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
  //       spinDistance(this::distanceToSpeaker).asProxy(),
  //       sequence(
  //           m_drivebaseS.resetPoseToBeginningC(path),
  //           // too-long intake delay
  //           autoIntakeCycle("C5 (1).1", 2, false, 1, ()->false),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5 (1).2", 0.2, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5 (1).3", 0.2, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5 (1).4", 0.2, true, 0.75, this::notAtMidline),
  //           feed().asProxy().withTimeout(0.3)));
  // }

  // public Command c5ThruStageRed() {
  //   return parallel(
  //       m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
  //       spinDistance(this::distanceToSpeaker).asProxy(),
  //       sequence(
  //           m_drivebaseS.choreoCommand("C5Red.1"),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5Red.2", 1, true, 0.75, ()->false),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5Red.3", 0.2, true, 0.75, ()->false),
  //           feed().asProxy().withTimeout(0.3),
  //           autoIntakeCycle("C5Red.4", 0.2, true, 0, ()->false),
  //           feed().asProxy().withTimeout(0.3)));
  // }

  // public Command c5ThruStage() {
  //   return Commands.either(
  //       c5ThruStageBlue(),
  //       c5ThruStageBlue(), AllianceWrapper::isBlue);
  // }

  // public Command disruptor() {
  //   var path = PathPlannerPath.fromChoreoTrajectory("disruptor").getTrajectory(ZERO_CHASSISSPEEDS, ZERO_ROTATION2D);
  //   return sequence(
  //       m_drivebaseS.resetPoseToBeginningC(path),
  //       m_drivebaseS.choreoCommand("disruptor"),
  //       m_drivebaseS.stopOnceC()).alongWith(m_midtakeS.runVoltage(() -> -2, () -> -2).asProxy());
  // }
}
