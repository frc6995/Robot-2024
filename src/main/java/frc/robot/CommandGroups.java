package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.amp.AmpRollerS;
import frc.robot.subsystems.amp.pivot.AmpPivotS;
import frc.robot.subsystems.bounceBar.BounceBarS;
import frc.robot.subsystems.climber.ClimberS;
import frc.robot.subsystems.drive.DrivebaseS;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.IntakeRollerS;
import frc.robot.subsystems.intake.IntakeRollerS.IR;
import frc.robot.subsystems.intake.pivot.IntakePivotS;
import frc.robot.subsystems.intake.pivot.IntakePivotS.IP;
import frc.robot.subsystems.shooter.Interpolation;
import frc.robot.subsystems.shooter.midtake.MidtakeS;
import frc.robot.subsystems.shooter.pivot.ShooterPivotS;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsS;
import frc.robot.subsystems.vision.BlobDetectionCamera;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import monologue.Annotations.Log;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import java.util.function.DoubleConsumer;

public class CommandGroups {


    private boolean autonIntake = false;
    private boolean straightThrough = false;
    private boolean autonShoot = false;
    private boolean driverRumbled = false;
    private boolean manipulatorRumbled = false;
    private boolean preload = false;

    public final EventLoop sensorEventLoop = new EventLoop();
    public final EventLoop stateEventLoop = new EventLoop();

    /** true = driver wants to intake */
    private final Trigger trg_driverIntakeReq;
    private final Trigger trg_driverCancelIntakeReq;
    /** true = driver wants to shoot */
    private final Trigger trg_driverShootReq;
    private final Trigger trg_driverAmpReq;
    private final Trigger trg_driverTrapReq;

    private final Trigger trg_autonIntakeReq = new Trigger(() -> autonIntake);
    private final Trigger trg_straightThroughReq = new Trigger(() -> straightThrough);
    private final Trigger trg_autonShootReq = new Trigger(() -> autonShoot);

    private final Trigger trg_preloadShootReq = new Trigger(() -> preload).and(RobotModeTriggers.autonomous());

    /** true = has note */

    public final Trigger trg_spunUp;
    public final Trigger trg_atAngle;
    public final Trigger trg_atPreloadAngle;

    private final Trigger trg_subwooferAngle;
    private final Trigger trg_ampAngle;

    private final Trigger trg_intakeReq;
    private final Trigger trg_shootReq;

    private final Trigger trg_midtakeNote;
    private final Trigger trg_overrideTOF;
    private final Trigger trg_atMidline;
    private final DigitalInput m_coastButton = new DigitalInput(0);
    private final Trigger trg_onbotCoastButton = new Trigger(m_coastButton::get).negate();
    private final Trigger trg_coastMode;
  private enum NoteState {
    EMPTY,
    FULL,//sitting in midtake
    INTAKE,// expecting a note to be intaken and hit beambreak
    EJECT, // slowly push note out of front of midtake
    SHOOTING,
    AMP_HANDOFF, // push note out of midtake, through shooter, to amp
    AMP_FULL; // note held entirely in amp rollers

    private NoteState() {
    }
  }
  private StateMachine<NoteState> m_noteSM = new StateMachine<CommandGroups.NoteState>(NoteState.EMPTY);

  // Midtake State (includes feeder roller)
  private enum MS {
    RECEIVE,// expecting a note to be intaken and hit beambreak
    PULL_IN, //continuing to bring the note in
    ADJUST, // reversing note back to beam break,
    EJECT,
    FEED,
    FEED_AMP,
    IDLE
  }
  private StateMachine<MS> midSM = new StateMachine<CommandGroups.MS>(MS.IDLE, MS.IDLE);
  // Intake Pivot


  private Swerve m_drivebaseS;
  private IntakePivotS m_intakePivotS;
  private IntakeRollerS m_intakeRollerS;
  private MidtakeS m_midtakeS;
  private AmpPivotS m_ampPivotS;
  private AmpRollerS m_ampRollerS;
  private ShooterPivotS m_shooterPivotS;
  private ShooterWheelsS m_shooterWheelsS;
  // private ClimberS m_climberS;
  private BlobDetectionCamera m_noteCamera;
  private LightStripS m_lightStripS;
  private CommandXboxController m_driverController;
  private CommandXboxController m_operatorController;

  public CommandGroups(
      Swerve drivebaseS,
      BlobDetectionCamera noteCamera,
      IntakePivotS intakePivotS,
      IntakeRollerS intakeRollerS,
      MidtakeS midtakeS,
      AmpPivotS ampPivotS,
      AmpRollerS ampRollerS,

      ShooterPivotS shooterPivotS,
      ShooterWheelsS shooterWheelsS,
      // ClimberS climberS,
      LightStripS lightStripS,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Trigger intaking, Trigger cancelIntake, Trigger shooting, Trigger ampShot, Trigger trapShot,
        DoubleConsumer driverRumbler, DoubleConsumer manipRumbler) {
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

        trg_driverIntakeReq = intaking;
        trg_driverCancelIntakeReq = cancelIntake;
        trg_driverShootReq = shooting;
        trg_driverAmpReq = ampShot;
        trg_driverTrapReq = trapShot;

        trg_subwooferAngle = new Trigger(() -> MathUtil.isNear(ShooterPivotS.Constants.SUBWOOFER_ANGLE, m_shooterPivotS.getAngle(), Units.degreesToRadians(2)));
        trg_ampAngle = new Trigger(() -> MathUtil.isNear(ShooterPivotS.Constants.AMP_ANGLE, m_shooterPivotS.getAngle(), Units.degreesToRadians(2)));


        trg_intakeReq = trg_driverIntakeReq.or(trg_autonIntakeReq).or(trg_straightThroughReq);
        trg_shootReq = trg_driverShootReq.or(trg_autonShootReq).or(trg_preloadShootReq);
        trg_midtakeNote = m_midtakeS.hasNote;
        trg_atMidline = new Trigger(this::notAtMidline).negate();
        trg_overrideTOF = (
          RobotModeTriggers.autonomous().and(trg_atMidline.negate())
        ).or(
          RobotModeTriggers.teleop().and(trg_driverIntakeReq)
        );

        
        trg_coastMode = trg_onbotCoastButton.or(m_driverController.back()).or(m_operatorController.back()).and(DriverStation::isDisabled);

        // irqTrg_frontSensor = new Trigger(sensorEventLoop, () -> frontVisiSightSeenNote);
        
        // // initialize (inverted)
        // shooterBeamBreakIrq = !shooterBeamBreak.get();

        // irqTrg_conveyorBeamBreak = new Trigger(sensorEventLoop, () -> conveyorBeamBreakIrq);
        // irqTrg_conveyorBeamBreak
        //     .onTrue(Commands.runOnce(()-> log_conveyorBeamBreakExtended.accept(true)).ignoringDisable(true));
        // irqTrg_conveyorBeamBreak.negate().debounce(0.1)
        //     .onTrue(Commands.runOnce(() -> log_conveyorBeamBreakExtended.accept(false)).ignoringDisable(true));

        // irqTrg_shooterBeamBreak = new Trigger(sensorEventLoop, () -> shooterBeamBreakIrq);
        // irqTrg_shooterBeamBreak
        //     .onTrue(Commands.runOnce(()-> log_shooterBeamBreakExtended.accept(true)).ignoringDisable(true));
        // irqTrg_shooterBeamBreak.negate().debounce(0.1)
        //     .onTrue(Commands.runOnce(() -> log_shooterBeamBreakExtended.accept(false)).ignoringDisable(true));

        trg_spunUp = m_shooterWheelsS.leftAtGoal.and(m_shooterWheelsS.rightAtGoal).debounce(0.05);
        trg_atAngle = m_shooterPivotS.atGoal;
        trg_atPreloadAngle = new Trigger(()->m_shooterPivotS.atGoal(ShooterPivotS.Constants.PRELOAD_TOLERANCE)).and(RobotModeTriggers.autonomous());

        // configureStateTriggers();

        // configureShootTimer();
  }

  public void configureMidtakeCommands() {
    trg_intakeReq
      .onTrue(midSM.setState(MS.RECEIVE));
    RobotModeTriggers.disabled()
      .onTrue(midSM.setState(MS.IDLE));
    midSM.trg(MS.IDLE)
      .whileTrue(m_midtakeS.stopC());
    midSM.trg(MS.RECEIVE)
      .onTrue(m_midtakeS.intakeC());
    midSM.trg(MS.PULL_IN)
      .onTrue(m_midtakeS.intakeC());
    midSM.trg(MS.ADJUST)
      .onTrue(m_midtakeS.backupC());
    midSM.trg(MS.EJECT)
      .onTrue(m_midtakeS.backupC());
    midSM.trg(MS.FEED)
      .onTrue(m_midtakeS.feedC());
    // state transition logic
    midSM.trg(MS.RECEIVE).and(trg_midtakeNote).and(trg_overrideTOF.negate())
      .onTrue(midSM.setState(MS.PULL_IN));
    midSM.trg(MS.PULL_IN).and(trg_midtakeNote.negate())
      .onTrue(midSM.setState(MS.ADJUST));
    midSM.trg(MS.ADJUST).and(trg_midtakeNote)
      .onTrue(midSM.setState(MS.IDLE));
    // timeouts
    Trigger pullInTimeout = midSM.trg(MS.PULL_IN).debounce(1);
    Trigger reverseTimeout = midSM.trg(MS.ADJUST).debounce(3);
    pullInTimeout.or(reverseTimeout)
      .onTrue(midSM.setState(MS.IDLE));
      
    
  }
  public void configureIntakeCommands() {

    Trigger driverCancelIntake = 
      midSM.trg(MS.RECEIVE).and(trg_driverCancelIntakeReq);
    Trigger driverRetract = 
      (midSM.trg(MS.PULL_IN).or(midSM.trg(MS.ADJUST))).and(trg_driverCancelIntakeReq);
    Trigger pullInTimeout = midSM.trg(MS.PULL_IN).debounce(1);
    Trigger reverseTimeout = midSM.trg(MS.ADJUST).debounce(3);
    // coast behavior for intake pivot
    trg_coastMode
      .onTrue(
        m_intakePivotS.SM.setState(IP.COAST)
      ).onFalse(
        m_intakePivotS.SM.clearState()
      );
      trg_intakeReq
      .onTrue(
        m_intakePivotS.SM.setState(IP.DEPLOY)
      )
      .onTrue(
        m_intakeRollerS.SM.setState(IR.FAST_IN)
      );
    // if driver pushes cancel intake while 


    driverRetract.onTrue(
      m_intakeRollerS.SM.setState(IR.STOP)
    ).onTrue(
      m_intakePivotS.SM.setState(IP.RETRACT)
    );
    driverCancelIntake
    .onTrue(
        either(m_noteSM.setState(NoteState.FULL), m_noteSM.setState(NoteState.EMPTY), trg_midtakeNote)
    );
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
                m_intakePivotS.deploy(),
                waitSeconds(0.2).andThen(m_intakeRollerS.intakeC()),
                waitSeconds(0.1).andThen(
                    m_midtakeS.intakeC()))
                .until(m_midtakeS.hasNote.and(overrideTOF.negate())),

            parallel(
                new ScheduleCommand(rumbleDriver(0.7).withTimeout(0.75)),
                new ScheduleCommand(m_lightStripS.stateC(() -> States.IntakedNote).withTimeout(0.75)
                  .andThen(m_lightStripS.stateC(()->States.HasNote).withTimeout(1.5))),
                sequence(
                    waitSeconds(0.5),
                    m_intakePivotS.retract()),
                m_intakeRollerS.slowInC(),
                m_midtakeS.intakeC())
                .withTimeout(1)
                .until(hasNote.negate())
                .onlyIf(hasNote),
            // intentionally interrupt the current command to fragment the group
            parallel(
                new ScheduleCommand(m_intakeRollerS.slowInC().withTimeout(1).andThen(m_intakeRollerS.stopC())),
                new ScheduleCommand(m_intakePivotS.retract()),
                new ScheduleCommand(
                    
                        m_midtakeS.backupC()
                        .withTimeout(3)
                        .until(hasNote)
                        .andThen(parallel(
                            m_midtakeS.stopOnceC())))))

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
        speaker()).getRadians() + Units.degreesToRadians(6);
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
    return 
        m_midtakeS.feedC();
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

  public Command w1() {
    var path = PathPlannerPath.fromChoreoTrajectory("W1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return deadline(
      sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            Pathing.setRotationOverride(()->Optional.of(new Rotation2d(this.directionToSpeaker()))),
            m_drivebaseS.choreoCommand("W1"),
            m_drivebaseS.stopOnceC(),
            waitSeconds(1),
            feed().asProxy().withTimeout(5)
      ),
      m_shooterPivotS.rotateWithVelocity(
            this::pivotAngle,
            () -> Interpolation.dThetadX(distanceToSpeaker()) *
                -Pathing.velocityTorwardsSpeaker(
                    m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
                    speaker()))
            .asProxy(),

        spinDistance(this::distanceToSpeaker).asProxy()

    ).finallyDo(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
  }
  public Command centerFourWingNote(double endWait) {
    var path = PathPlannerPath.fromChoreoTrajectory("W2.1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return deadline(
        sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            Pathing.setRotationOverride(()->Optional.of(new Rotation2d(this.directionToSpeaker()))),
            m_drivebaseS.choreoCommand("W2.1"),
            m_drivebaseS.stopOnceC(),
            feed().asProxy().withTimeout(0.75),
            deadline(
                sequence(
                    m_drivebaseS.choreoCommand("W2.2"),
                    m_drivebaseS.stopOnceC(),
                    waitSeconds(0),
                    m_drivebaseS.choreoCommand("W2.3"),
                    m_drivebaseS.stopOnceC(),
                    waitSeconds(0),
                    m_drivebaseS.choreoCommand("W2.4"),
                    m_drivebaseS.stopOnceC(),
                    Pathing.clearRotationOverride(),
                    waitSeconds(endWait)),
                m_intakeRollerS.intakeC().asProxy(),
                feed().asProxy()
            )
        ),
        m_intakePivotS.deploy().asProxy(),
        m_shooterPivotS.rotateWithVelocity(
            this::pivotAngle,
            () -> Interpolation.dThetadX(distanceToSpeaker()) *
                -Pathing.velocityTorwardsSpeaker(
                    m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
                    speaker()))
            .asProxy(),

        spinDistance(this::distanceToSpeaker).asProxy()

    ).finallyDo(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
  }

  public Command spinDistance(DoubleSupplier distance) {
    return m_shooterWheelsS.spinC(() -> Interpolation.LEFT_MAP.get(distance.getAsDouble()),
        () -> Interpolation.RIGHT_MAP.get(distance.getAsDouble()));
  }

  public Command centerFourWingMidline() {
    return sequence(
        centerFourWingNote(0.25),
        parallel(
            sequence(
                autoIntakeCycle("W2.5", 1, true, 0.75, this::notAtMidline),
                m_drivebaseS.stopOnceC(),
                feed().asProxy().withTimeout(0.25),
                autoIntakeCycle("W2.6", 0.25, true, 0.75, this::notAtMidline),
                m_drivebaseS.stopOnceC(),
                feed().asProxy().withTimeout(5)
            ),
            m_shooterPivotS.rotateWithVelocity(
                this::pivotAngle,
                () -> Interpolation.dThetadX(distanceToSpeaker()) *
                    -Pathing.velocityTorwardsSpeaker(
                        m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
                        speaker()))
                .asProxy(),
            spinDistance(this::distanceToSpeaker).asProxy()));
  }
  public Command centerFourWingC3C4() {
    return sequence(
        centerFourWingNote(0.25),
        parallel(
            sequence(
                autoIntakeCycle("4Close-C3.1", 0.5, true, 0.75, this::notAtMidline),
                m_drivebaseS.stopOnceC(),
                feed().asProxy().withTimeout(0.25),
                autoIntakeCycle("4Close-C3.2", 0.25, true, 0.75, this::notAtMidline),
                m_drivebaseS.stopOnceC(),
                feed().asProxy().withTimeout(5)
            ),
            m_shooterPivotS.rotateWithVelocity(
                this::pivotAngle,
                () -> Interpolation.dThetadX(distanceToSpeaker()) *
                    -Pathing.velocityTorwardsSpeaker(
                        m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
                        speaker()))
                .asProxy(),
            spinDistance(this::distanceToSpeaker).asProxy()));
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

  public Command autoIntakeCycle(String choreoTrajectory, double intakeTimeout, boolean feed, double aimTime, BooleanSupplier noNoteYet) {
    var path = PathPlannerPath.fromChoreoTrajectory(choreoTrajectory);
    var traj = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
    var startAimTime = traj.getTotalTimeSeconds() - aimTime;
    return deadline(

        sequence(
            m_drivebaseS.pathPlannerCommand(path),
            m_drivebaseS.stopOnceC()

        ),
        (aimTime > 0 ? waitSeconds(startAimTime).andThen(
            Pathing.setRotationOverride(
                () -> Optional.of(new Rotation2d(this.directionToSpeaker()))))
            : none()),
        (feed ? feed().asProxy().withTimeout(intakeTimeout) : waitSeconds(intakeTimeout)).andThen(
            deployRunIntake(new Trigger(noNoteYet)).asProxy()))
        .finallyDo(
            () -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
  }

  public boolean notAtMidline() {
    return Math.abs(m_drivebaseS.getPose().getX() - 8.22) > 0.5;
  }
  public Command c5() {
    var path = PathPlannerPath.fromChoreoTrajectory("C5S.1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return parallel(
        new ScheduleCommand(m_intakePivotS.deploy().asProxy()),
        m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
        spinDistance(this::distanceToSpeaker).asProxy(),
        sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            autoIntakeCycle("C5S.1", 3, false, 1, ()->false),
            feed().asProxy().withTimeout(0.5),
            autoIntakeCycle("C5S.2", 1, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.5),
            autoIntakeCycle("C5S.3", 0.2, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.5)));
  }

  public Command c5ThruStageBlue() {
    var path = PathPlannerPath.fromChoreoTrajectory("C5.1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return parallel(
        m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
        spinDistance(this::distanceToSpeaker).asProxy(),
        sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            // too-long intake delay
            autoIntakeCycle("C5.1", 2, false, 1, ()->false),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5.2", 1, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5.3", 0.2, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5.4", 0.2, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.3)));
  }

  public Command c4c3() {
    var path = PathPlannerPath.fromChoreoTrajectory("C5 (1).1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return parallel(
        m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
        spinDistance(this::distanceToSpeaker).asProxy(),
        sequence(
            m_drivebaseS.resetPoseToBeginningC(path),
            // too-long intake delay
            autoIntakeCycle("C5 (1).1", 2, false, 1, ()->false),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5 (1).2", 0.2, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5 (1).3", 0.2, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5 (1).4", 0.2, true, 0.75, this::notAtMidline),
            feed().asProxy().withTimeout(0.3)));
  }

  public Command c5ThruStageRed() {
    return parallel(
        m_shooterPivotS.rotateToAngle(this::pivotAngle).asProxy(),
        spinDistance(this::distanceToSpeaker).asProxy(),
        sequence(
            m_drivebaseS.choreoCommand("C5Red.1"),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5Red.2", 1, true, 0.75, ()->false),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5Red.3", 0.2, true, 0.75, ()->false),
            feed().asProxy().withTimeout(0.3),
            autoIntakeCycle("C5Red.4", 0.2, true, 0, ()->false),
            feed().asProxy().withTimeout(0.3)));
  }

  public Command c5ThruStage() {
    return Commands.either(
        c5ThruStageBlue(),
        c5ThruStageBlue(), AllianceWrapper::isBlue);
  }

  public Command disruptor() {
    var path = PathPlannerPath.fromChoreoTrajectory("disruptor").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    return sequence(
        m_drivebaseS.resetPoseToBeginningC(path),
        m_drivebaseS.choreoCommand("disruptor"),
        m_drivebaseS.stopOnceC()).alongWith(m_midtakeS.runVoltage(() -> -2, () -> -2, ()->0).asProxy());
  }
}
