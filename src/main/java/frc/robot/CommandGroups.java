package frc.robot;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.amp.AmpRollerS;
/*Changed the name of the import to match the document name change. */
import frc.robot.subsystems.amp.pivot.CTREAmpPivotS;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.IntakeRollerS;
import frc.robot.subsystems.intake.pivot.IntakePivotS;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.LightStripS.States;
import frc.robot.subsystems.shooter.Interpolation;
import frc.robot.subsystems.shooter.midtake.MidtakeS;
import frc.robot.subsystems.shooter.pivot.ShooterPivotS;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsS;
import frc.robot.subsystems.vision.BlobDetectionCamera;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import static frc.robot.util.Defaults.*;

public class CommandGroups {
  private Swerve m_drivebaseS;
  private IntakePivotS m_intakePivotS;
  private IntakeRollerS m_intakeRollerS;
  /* Changed the name to match the document name change. */
  private MidtakeS m_midtakeS;
  private CTREAmpPivotS m_ampPivotS;
  private AmpRollerS m_ampRollerS;
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
      /* Changed the name to match the document name change. */
      CTREAmpPivotS ampPivotS,
      AmpRollerS ampRollerS,
      ShooterPivotS shooterPivotS,
      ShooterWheelsS shooterWheelsS,
      // ClimberS climberS,
      LightStripS lightStripS,
      CommandXboxController driverController,
      TrajectoryLogger<SwerveSample> trajectoryLogger) {
    m_drivebaseS = drivebaseS;
    m_intakePivotS = intakePivotS;
    m_intakeRollerS = intakeRollerS;
    m_midtakeS = midtakeS;
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
        (interrupted, pose, sample) -> {
          m_drivebaseS.stop();
        },
        AllianceWrapper::isRed,
        new AutoFactory.AutoBindings(),
        trajectoryLogger);

    // Triggers
    pivotsHandoffReady = m_shooterPivotS.atAmpAngle.and(m_ampPivotS.atHandoffAngle);
    ampReceivesNote = m_ampRollerS.receiveNote;
    ampPivotUp = m_ampPivotS.outOfShooter;
  }

  public final Trigger pivotsHandoffReady,
      ampPivotUp,
      ampReceivesNote;

  /**
   * Requires: None
   * Schedules: Shooter Pivot, Amp Pivot, Amp Roller
   * Schedules After Pivots Ready: Shooter Wheels, Midtake
   */
  public Command loadAmp(BooleanSupplier cancel, DoubleSupplier finalAmpPosition) {
    return
    // shooter pivot
    new ScheduleCommand(
        sequence(
            m_shooterPivotS.handoffAngle().until(ampReceivesNote),
            m_shooterPivotS.handoffAngle().until(ampPivotUp)).until(cancel),
        sequence(
            m_ampPivotS.handoffAngle().until(ampReceivesNote),
            parallel(m_ampPivotS.rotateToAngle(finalAmpPosition).withTimeout(1.5),
                either(
                    new ScheduleCommand(m_intakeRollerS.runVoltageC(() -> 1)
                        .withTimeout(1.5)),
                    none(), () -> finalAmpPosition.getAsDouble() < Units.degreesToRadians(30))))
            .until(cancel),

        sequence(
            m_ampRollerS.intakeC().until(ampReceivesNote),
            m_ampRollerS.intakeC().withTimeout(0.4)).until(cancel),
        parallel(waitUntil(pivotsHandoffReady), waitSeconds(0.1)).andThen(
            new ScheduleCommand(
                sequence(
                    m_shooterWheelsS.spinC(() -> 3000, () -> 3000).until(ampReceivesNote),
                    m_shooterWheelsS.spinC(() -> 1500, () -> 1500).withTimeout(1)).until(cancel),
                sequence(
                    waitSeconds(0.1),
                    m_midtakeS.outtakeC().until(ampReceivesNote)).until(cancel)))
            .until(cancel));
  }

  /**
   * Requires: Amp Roller, Amp Pivot
   * 
   * @return
   */
  public Command scoreAmp() {
    return sequence(
        deadline(
            sequence(
                m_ampRollerS.stopC().until(m_ampPivotS.atScore).withTimeout(1.5),
                m_ampRollerS.outtakeC().withTimeout(0.5),
                m_ampRollerS.stopOnceC()),
            m_ampPivotS.score()),
        m_ampPivotS.stow().withTimeout(1.5));
  }

  /**
   * Requires: None
   * 
   * @param cancel
   * @param finalAmpPosition
   * @param overrideTOF
   * @return
   */
  public Command intakeLoadAmp(BooleanSupplier cancel, DoubleSupplier finalAmpPosition, Trigger overrideTOF) {
    return either(
        new ScheduleCommand(loadAmp(cancel, finalAmpPosition)),
        parallel(
            loadAmp(cancel, finalAmpPosition),
            // This will likely get interrupted by the loadAmp
            new ScheduleCommand(midtakeReceiveNote(overrideTOF).until(cancel)),
            deployRunIntakeOnly(overrideTOF).until(cancel).andThen(retractStopIntake())),
        m_midtakeS.hasNote);
  }

  /**
   * Requires: Intake Pivot
   */
  public Command deployIntakeUntilNote(Trigger hasNote) {
    return sequence(
        m_intakePivotS.deploy().until(hasNote),
        m_intakePivotS.rotateToAngle(() -> IntakePivotS.Constants.CW_LIMIT + Units.degreesToRadians(15))
            .withTimeout(0.3));
  }

  /**
   * Requires: IntakeRoller
   */
  public Command runIntakeUntilNote(Trigger hasNote) {
    return sequence(
        m_intakeRollerS.intakeC().until(hasNote),
        sequence(
            m_intakeRollerS.slowInC()
                .withTimeout(0.2)
                .until(hasNote.negate()),
            m_intakeRollerS.slowInC().withTimeout(0.1)).onlyIf(hasNote));
  }

  /**
   * Requires: IntakeRoller, Intake Pivot
   */
  public Command deployRunIntakeOnly(Trigger overrideTOF) {
    Trigger hasNote = m_midtakeS.hasNote.and(overrideTOF.negate());
    return parallel(
        // intake pivot
        deployIntakeUntilNote(hasNote),
        // intake roller
        waitUntil(() -> m_intakePivotS.getAngle() < 0).andThen(runIntakeUntilNote(hasNote)));
  }

  /**
   * Requires: None
   * 
   * @return
   */
  public Command intakeNoteFeedback() {
    return new ScheduleCommand(
        rumbleDriver(0.7).withTimeout(0.75),
        m_lightStripS.stateC(() -> States.IntakedNote).withTimeout(0.75)
            .andThen(m_lightStripS.stateC(() -> States.HasNote).withTimeout(1.5)));
  }

  /**
   * Requires: Midtake
   * 
   * @param overrideTOF
   * @return
   */
  public Command midtakeReceiveNote(Trigger overrideTOF) {
    Trigger hasNote = m_midtakeS.hasNote.and(overrideTOF.negate());

    return sequence(
        m_midtakeS.intakeC().until(hasNote),
        either(
            sequence(
                parallel(
                    intakeNoteFeedback(),
                    m_midtakeS.intakeC().until(hasNote.negate()).withTimeout(1)),
                m_midtakeS.reverseC()
                    .withTimeout(1)
                    .until(m_midtakeS.hasNote).finallyDo(m_midtakeS::stop)),
            m_midtakeS.stopOnceC(),
            m_midtakeS.hasNote));
  }

  /**
   * This command deploys the intake, runs the intake rollers, and prepares to
   * receive the
   * note in the midtake.
   * 
   * End: When note is in midtake
   */
  public Command deployRunIntake(Trigger overrideTOF) {
    return parallel(
        deployRunIntakeOnly(overrideTOF),
        midtakeReceiveNote(overrideTOF));
  }

  public Command retractStopIntake() {
    return parallel(
        m_intakePivotS.runOnce(() -> {
        }).asProxy(),
        either(waitSeconds(0.5), none(), m_midtakeS.hasNote).andThen(
            m_intakeRollerS.runOnce(() -> {
            }).asProxy()));
  }

  // public Command autoPickupC() {
  // return defer(() -> {
  // var note = m_noteCamera.getBestTarget(m_drivebaseS.getPose());
  // if (note.isEmpty()) {
  // return Commands.none();
  // } // TODO driver feedback? Must be proxied for duration
  // var pickup = note.get().transformBy(new Transform2d(new Translation2d(-1, 0),
  // ZERO_ROTATION2D));
  // return parallel(
  // sequence(
  // m_drivebaseS.chasePoseC(() -> pickup),
  // m_drivebaseS.run(() -> {
  // m_drivebaseS.drive(new ChassisSpeeds(0.5, 0, 0));
  // })),
  // deployRunIntake(new Trigger(() -> false)));
  // }, Set.of(m_drivebaseS, m_intakePivotS, m_intakeRollerS));
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

  public double autoDistanceToSpeaker() {
    return Math.min(distanceToSpeaker(), Interpolation.PASSING_DISTANCE - 0.05);
  }

  public double pivotAngle() {
    return Interpolation.PIVOT_MAP.get(distanceToSpeaker());
  }

  public Command feed() {
    return m_midtakeS.outtakeC();
  }

  public Command faceSpeaker(DoubleSupplier fwdX, DoubleSupplier fwdY) {
    return m_drivebaseS.manualFieldHeadingDriveC(fwdX, fwdY,
        this::directionToSpeaker,
        () -> Pathing.aimingFFVelocity(
            m_drivebaseS.getPose(),
            m_drivebaseS.getFieldRelativeLinearSpeedsMPS(), NomadMathUtil.mirrorTranslation(
                Constants.Poses.SPEAKER,
                AllianceWrapper.getAlliance())))
        .alongWith(
            run(() -> {
              var error = m_drivebaseS.getPoseHeading().minus(new Rotation2d(this.directionToSpeaker())).getRadians();
              if (error >= Units.degreesToRadians(1)) {
                // drivetrain is too far ccw, light the left third of the bar
                LightStripS.getInstance().requestState(States.LeftThird);
              } else if (error <= Units.degreesToRadians(-1)) {
                LightStripS.getInstance().requestState(States.RightThird);
              } else {
                LightStripS.getInstance().requestState(States.CenterThird);
              }
            }));
  }

  /** AUTOS */
  public record AutoRoutine(Command cmd, Pose2d[] poses, Pose2d[] flippedPoses, double estTime) {
    Pose2d[] getPoses() {
      return AllianceWrapper.isRed() ? flippedPoses() : poses();
    }

    Optional<Pose2d> getStart() {
      var poses = getPoses();
      if (poses.length > 0) {
        return Optional.of(poses[0]);
      }
      return Optional.empty();
    }
  }

  public final Trigger notAtMidline = new Trigger(this::notAtMidline);

  public void addAutoRoutines(SendableChooser<AutoRoutine> chooser) {
    chooser.setDefaultOption("Do Nothing", new AutoRoutine(none(), new Pose2d[0], new Pose2d[0], 0));
    // chooser.addOption("O-C213", O_C213());
    chooser.addOption("4Note", O_C231());
    // chooser.addOption("O-C213-M3", O_C213_M3());
    chooser.addOption("4Note-Mid", O_C231_M3());
    // chooser.addOption("O-C2-M3-C13", O_C2_M3_C13());
    chooser.addOption("Mid-4Note", O_M3_C321());
    // chooser.addOption("D-M12-P", D_M12_P());
    chooser.addOption("AmpRush-Wall", O_M21());
    chooser.addOption("AmpRush-Center", O_M23());
  }

  public Command s2Toc2() {

    var loop = m_autoFactory.newLoop("S2-C2");
    var trajectory = m_autoFactory.trajectory("S2-C2", loop);
    loop.enabled().onTrue(m_drivebaseS.resetPoseToBeginningC(trajectory));
    loop.enabled().onTrue(trajectory.cmd());
    return loop.cmd();
  }

  private static final double FIRST_SHOT_PAUSE = 0.75;

  public void firstMove(AutoLoop loop, AutoTrajectory first) {
    loop.enabled()
        .onTrue(m_drivebaseS.resetPoseToBeginningC(first));
    first.atTime(0.1).onTrue(m_intakePivotS.deploy());
    first.atTime(0).onTrue(spinDistance(this::autoDistanceToSpeaker));
  }

  public void firstShot(AutoTrajectory first) {
    first.done().onTrue(feed());
  }

  public Command endAuto() {
    return parallel(
        new ScheduleCommand(m_shooterWheelsS.stopOnceC()),
        new ScheduleCommand(m_intakePivotS.runOnce(() -> {
        })),
        new ScheduleCommand(m_intakeRollerS.stopOnceC()),
        new ScheduleCommand(m_midtakeS.stopOnceC()),
        new ScheduleCommand(m_drivebaseS.stopOnceC()));
  }

  private Pose2d[] poses(boolean flipped, AutoTrajectory... trajectories) {
    List<Pose2d> poseList = new ArrayList<>();
    for (AutoTrajectory t : trajectories) {
      poseList.addAll(Arrays.asList(flipped ? t.trajectoryFlipped.getPoses() : t.trajectory.getPoses()));
    }
    return poseList.toArray(new Pose2d[0]);
  }

  private AutoRoutine routine(Command command, double extraTime, AutoTrajectory... trajectories) {
    double time = extraTime;
    for (AutoTrajectory t : trajectories) {
      time += t.trajectory.getTotalTime();
    }
    return new AutoRoutine(
        command,
        poses(false, trajectories),
        poses(true, trajectories),
        time);
  }

  public Command shotPause() {
    return faceSpeaker(() -> 0, () -> 0);
  }

  public AutoRoutine O_C213() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S2-SH2", loop);
    var second = m_autoFactory.trajectory("SH2-C2", loop);
    var third = m_autoFactory.trajectory("C2-C1", loop);
    var fourth = m_autoFactory.trajectory("C1-C3", loop);
    firstMove(loop, first);
    firstShot(first);
    loop.enabled().onTrue(
        sequence(
            first.cmd(),
            shotPause().withTimeout(FIRST_SHOT_PAUSE),
            second.cmd(),
            shotPause().withTimeout(0.75),
            third.cmd(),
            shotPause().withTimeout(0.75),
            fourth.cmd(),
            shotPause().withTimeout(8)));
    second.atTime(0).onTrue(m_intakeRollerS.intakeC()).onTrue(feed());

    return routine(
        loop.cmd(),
        FIRST_SHOT_PAUSE,
        first, second, third, fourth);
  }

  public AutoRoutine O_C213_M3() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S2-SH2", loop);
    var SH2_pause = FIRST_SHOT_PAUSE;
    var second = m_autoFactory.trajectory("SH2-C2", loop);
    var C2_pause = 0.75;
    var third = m_autoFactory.trajectory("C2-C1", loop);
    var C1_pause = 0.75;
    var fourth = m_autoFactory.trajectory("C1-C3", loop);
    var C3_pause = 0.75;
    var fifth = m_autoFactory.trajectory("C3-M3", loop);
    var sixth = m_autoFactory.trajectory("M3-SH3", loop);

    firstMove(loop, first);
    firstShot(first);

    loop.enabled().onTrue(
        sequence(
            first.cmd(),
            shotPause().withTimeout(SH2_pause),
            second.cmd(),
            shotPause().withTimeout(C2_pause),
            third.cmd(),
            shotPause().withTimeout(C1_pause),
            fourth.cmd(),
            shotPause().withTimeout(C3_pause),
            fifth.cmd()));
    second.atTime(0).onTrue(m_intakeRollerS.intakeC()).onTrue(feed());
    fifth.atTime("intake").onTrue(deployRunIntake(new Trigger(this::notAtMidline)));
    fifth.done().onTrue(sixth.cmd());
    sixth.atTime(0.9).onTrue(retractStopIntake());
    sixth.done().onTrue(feed());
    sixth.done().onTrue(m_drivebaseS.stopOnceC());

    return routine(
        loop.cmd(),
        C1_pause + C2_pause + C3_pause + SH2_pause,
        first, second, third, fourth, fifth, sixth);
  }

  public AutoRoutine O_C231() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S2-SH2", loop);
    var SH2_pause = FIRST_SHOT_PAUSE;
    var second = m_autoFactory.trajectory("SH2-C2", loop);
    var C2_pause = 0.75;
    var third = m_autoFactory.trajectory("C2-C3", loop);
    var C3_pause = 0.75;
    var fourth = m_autoFactory.trajectory("C3-C1", loop);
    firstMove(loop, first);
    firstShot(first);
    loop.enabled().onTrue(
        sequence(
            first.cmd(),
            shotPause().withTimeout(SH2_pause),
            second.cmd(),
            shotPause().withTimeout(C2_pause),
            third.cmd(),
            shotPause().withTimeout(C3_pause),
            fourth.cmd(),
            shotPause().withTimeout(8)));
    second.atTime(0).onTrue(m_intakeRollerS.intakeC()).onTrue(feed());

    return routine(
        loop.cmd(),
        FIRST_SHOT_PAUSE, first, second, third, fourth);
  }

  public AutoRoutine O_C231_M3() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S2-SH2", loop);
    var SH2_pause = FIRST_SHOT_PAUSE;
    var second = m_autoFactory.trajectory("SH2-C2", loop);
    var C2_pause = 0.75;
    var third = m_autoFactory.trajectory("C2-C3", loop);
    var C3_pause = 0.75;
    var fourth = m_autoFactory.trajectory("C3-C1", loop);
    var C1_pause = 0.75;
    var fifth = m_autoFactory.trajectory("C1-M3", loop);
    var sixth = m_autoFactory.trajectory("M3-SH3", loop);

    firstMove(loop, first);
    firstShot(first);
    loop.enabled().onTrue(
        sequence(
            first.cmd(),
            shotPause().withTimeout(SH2_pause),
            second.cmd(),
            shotPause().withTimeout(C2_pause),
            third.cmd(),
            shotPause().withTimeout(C3_pause),
            fourth.cmd(),
            shotPause().withTimeout(C1_pause),
            fifth.cmd()));
    second.atTime(0).onTrue(m_intakeRollerS.intakeC()).onTrue(feed());

    fifth.atTime("intake").onTrue(deployRunIntake(new Trigger(this::notAtMidline)));
    fifth.done().onTrue(sixth.cmd());
    sixth.atTime(1.2).onTrue(retractStopIntake());
    sixth.done().onTrue(feed());
    sixth.done().onTrue(m_drivebaseS.stopOnceC());

    return routine(
        loop.cmd(),
        FIRST_SHOT_PAUSE, first, second, third, fourth, fifth, sixth);
  }

  public AutoRoutine O_C2_M3_C13() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S2-SH2", loop);
    var SH2C2 = m_autoFactory.trajectory("SH2-C2", loop);
    var C2M3 = m_autoFactory.trajectory("C2-M3", loop);
    var M3C1 = m_autoFactory.trajectory("M3-C1", loop);
    var fifth = m_autoFactory.trajectory("C1-C3", loop);

    // Shoot O
    firstMove(loop, first);
    firstShot(first);
    loop.enabled().onTrue(
        sequence(
            first.cmd(),
            shotPause().withTimeout(FIRST_SHOT_PAUSE),
            SH2C2.cmd(),
            shotPause().withTimeout(FIRST_SHOT_PAUSE),
            C2M3.cmd(),
            M3C1.cmd(),
            shotPause().withTimeout(0.75),
            fifth.cmd(),
            shotPause().withTimeout(8)));
    // Go towards C2 with intake down, rollers going, shooter feeding
    SH2C2.atTime(0).onTrue(m_intakeRollerS.intakeC()).onTrue(feed());
    // Go to
    C2M3.atTime("intake").onTrue(deployRunIntake(new Trigger(this::notAtMidline)));
    M3C1.atTime(0.3).onTrue(retractStopIntake());
    M3C1.atTime("feed")
        .onTrue(feed())
        .onTrue(m_intakePivotS.deploy())
        .onTrue(m_intakeRollerS.intakeC());

    return routine(
        loop.cmd(),
        FIRST_SHOT_PAUSE, first, SH2C2, C2M3, M3C1, fifth);
  }

  public AutoRoutine O_M3_C321() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("S4-M3", loop);
    var M3SH5 = m_autoFactory.trajectory("M3-SH5", loop);
    var SH5C3 = m_autoFactory.trajectory("SH5-C3", loop);
    var C3C2 = m_autoFactory.trajectory("C3-C2", loop);
    var C2C1 = m_autoFactory.trajectory("C2-C1", loop);

    loop.enabled()
        .onTrue(m_drivebaseS.resetPoseToBeginningC(first));
    first.atTime("intake").onTrue(deployRunIntake(notAtMidline));
    first.atTime(0).onTrue(spinDistance(this::autoDistanceToSpeaker));
    first.atTime("feed").onTrue(feed());
    first.atTime("intake").onTrue(deployRunIntake(notAtMidline)).onTrue(midtakeReceiveNote(new Trigger(()->false)));
    loop.enabled().onTrue(
        sequence(
            first.cmd(),
            M3SH5.cmd(),
            shotPause().withTimeout(0.75),
            SH5C3.cmd(),
            shotPause().withTimeout(0.75),
            C3C2.cmd(),
            shotPause().withTimeout(0.75),
            C2C1.cmd(),
            shotPause().withTimeout(8)));
    first.done().onTrue(feed());

    M3SH5.done().onTrue(m_intakePivotS.deploy());
    M3SH5.done().onTrue(feed());
    M3SH5.done().onTrue(m_intakeRollerS.intakeC());

    return routine(
        loop.cmd(),
        0.75 * 3, first, M3SH5, SH5C3, C3C2, C2C1);
  }

  public static Trigger always (EventLoop loop) {
    return new Trigger(loop, ()->true);
  }
  public static Trigger never (EventLoop loop) {
    return new Trigger(loop, ()->false);
  }
  public AutoRoutine O_M21() {
    var loop = m_autoFactory.newLoop("O_M213");
    var first = m_autoFactory.trajectory("E1-M2R-LateShot", loop);
    var M2RSH4 = m_autoFactory.trajectory("M2R-SH4", loop);
    var SH4M1 = m_autoFactory.trajectory("SH4-M1", loop);
    var M1SH4 = m_autoFactory.trajectory("M1-SH4", loop);
    final double spinupTime = 0;
    loop.enabled()
      .onTrue(sequence(
        m_drivebaseS.resetPoseToBeginningC(first),
        waitSeconds(spinupTime),
        first.cmd(),
        M2RSH4.cmd(),
        shotPause().withTimeout(0.5),
        SH4M1.cmd(),
        M1SH4.cmd(),
        shotPause()
    ))
    .onTrue(
      spinDistance(this::autoDistanceToSpeaker)
    );
    first.atTime("intake").or(SH4M1.atTime("intake")).onTrue(midtakeReceiveNote(never(loop.getLoop()))).onTrue(
      deployRunIntakeOnly(notAtMidline)
    );
    first.atTime("feed").onTrue(feed());
    M2RSH4.done().onTrue(feed());
    M1SH4.done().onTrue(feed());

    return routine(
      loop.cmd(), 0.5+spinupTime, first, M2RSH4, SH4M1, M1SH4, SH4M1);
  }

  public AutoRoutine O_M23() {
    var loop = m_autoFactory.newLoop("O_M213");
    var first = m_autoFactory.trajectory("E1-M2R-LateShot", loop);
    var M2RSH4 = m_autoFactory.trajectory("M2R-SH4", loop);
    var SH4M3 = m_autoFactory.trajectory("SH4-M3", loop);
    var M3SH6 = m_autoFactory.trajectory("M3-SH6", loop);
    final double spinupTime = 0;
    loop.enabled()
      .onTrue(sequence(
        m_drivebaseS.resetPoseToBeginningC(first),
        waitSeconds(spinupTime),
        first.cmd(),
        M2RSH4.cmd(),
        shotPause().withTimeout(0.5),
        SH4M3.cmd(),
        M3SH6.cmd(),
        shotPause()
    ))
    .onTrue(
      spinDistance(this::autoDistanceToSpeaker)
    );
    first.atTime("intake").or(SH4M3.atTime("intake")).onTrue(midtakeReceiveNote(never(loop.getLoop()))).onTrue(
      deployRunIntakeOnly(notAtMidline)
    );
    first.atTime("feed").onTrue(feed());
    M2RSH4.done().onTrue(feed());
    M3SH6.done().onTrue(feed());

    return routine(
      loop.cmd(), 0.5+spinupTime, first, M2RSH4, SH4M3, M3SH6, SH4M3);
  }
  public AutoRoutine D_M12_P() {

    var loop = m_autoFactory.newLoop("FourNote");
    var first = m_autoFactory.trajectory("E1-M1", loop);
    var M1SH4 = m_autoFactory.trajectory("M1-SH4", loop);
    var SH4M2 = m_autoFactory.trajectory("SH4-M2", loop);
    var M2SH4 = m_autoFactory.trajectory("M2-SH4", loop);
    var SH4D1 = m_autoFactory.trajectory("SH4-D1", loop);
    var D1SH4 = m_autoFactory.trajectory("D1-SH4", loop);

    loop.enabled()
        .onTrue(m_drivebaseS.resetPoseToBeginningC(first))

        .onTrue(m_intakePivotS.deploy())
        .onTrue(
            sequence(
                first.cmd(),
                M1SH4.cmd(),
                shotPause().withTimeout(0.75),
                SH4M2.cmd(),
                M2SH4.cmd(),
                shotPause().withTimeout(0.75),
                SH4D1.cmd(),
                D1SH4.cmd(),
                shotPause().withTimeout(8)));

    // First path: drop onboard at wingline, intake M1
    first.atTime("drop").onTrue(m_shooterWheelsS.spinC(() -> 2000, () -> 2000).withTimeout(0.7)
        .andThen(spinDistance(this::autoDistanceToSpeaker)));
    first.atTime("intake").onTrue(deployRunIntake(notAtMidline));

    // Shoot M1
    M1SH4.atTime("feed")
        .onTrue(feed().withTimeout(1));
    // Pickup M2
    // offset startup of shooter wheels
    SH4M2.atTime("intake").onTrue(deployRunIntake(notAtMidline));
    // Shoot M2
    M2SH4.atTime("feed").onTrue(feed().withTimeout(1));
    // Pickup D1
    SH4D1.atTime("intake").onTrue(deployRunIntake(new Trigger(() -> false)));
    D1SH4.atTime("feed")
        .onTrue(feed().withTimeout(1));

    return routine(
        loop.cmd(),
        0, first, M1SH4, SH4M2, M2SH4, SH4D1, D1SH4);
  }

  public Command driveToPreAmp() {
    return m_drivebaseS.driveToPoseC(() -> Pathing.getOwnPreAmp());
  }

  public Command driveToAmp() {
    return m_drivebaseS.driveToPoseC(() -> Pathing.getOwnAmp());
  }

  public Command spinDistance(DoubleSupplier distance) {
    return m_shooterWheelsS.spinC(() -> Interpolation.LEFT_MAP.get(distance.getAsDouble()),
        () -> Interpolation.RIGHT_MAP.get(distance.getAsDouble()));
  }

  public boolean notAtMidline() {
    return Math.abs(m_drivebaseS.getPose().getX() - 8.22) > 0.5;
  }
}
