package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandGroups.AutoRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.amp.AmpRollerS;
import frc.robot.subsystems.amp.pivot.CTREAmpPivotS;
import frc.robot.subsystems.climber.ClimberS;
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
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkDevice;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import static frc.robot.util.Defaults.*;
public class RobotContainer implements Logged {

  /** Establishes the controls and subsystems of the robot */
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final CommandXboxController m_keypad = new CommandXboxController(2);
  private final DriverDisplay m_driverDisplay = new DriverDisplay();
  private final Swerve m_drivebaseS;

  @Log
  private final Mechanism2d MECH_VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
  private final ShooterFeederS m_shooterFeederS;
  private final ShooterPivotS m_shooterPivotS;
  private final ShooterWheelsS m_shooterWheelsS;
  private final IntakePivotS m_intakePivotS;
  private final IntakeRollerS m_intakeRollerS;
  private final MidtakeS m_midtakeS;
  /*Changed the name to match the document name change. */
  private final CTREAmpPivotS m_ampPivotS;
  private final AmpRollerS m_ampRollerS;
  private final ClimberS m_leftClimberS;
  private final ClimberS m_rightClimberS;
  private final BlobDetectionCamera m_noteCamera;
  private final LightStripS m_lightStripS;
  @Log
  private double loopTime = 0;
  private LinearFilter loopTimeAverage = LinearFilter.movingAverage(1);
  @Log.NT
  private final Field2d m_field = new Field2d();
  private final Field2d m_driverField = new Field2d();

  // private final PDData m_powerLogger = PDData.create(1,ModuleType.kRev);
  // @Log public PDData power() {return m_powerLogger.update();}
  private final CommandGroups m_autos;

  private InputAxis m_fwdXAxis = new InputAxis("Forward", m_driverController::getLeftY)
      .withInvert(true)
      .withSlewRate(3);
  private InputAxis m_fwdYAxis = new InputAxis("Strafe", m_driverController::getLeftX)
      .withInvert(true)
      .withSlewRate(3);
      
  private InputAxis m_rotAxis = new InputAxis("Rotate", m_driverController::getRightX)
      .withDeadband(0.2)
      .withInvert(true)
      .withSlewRate(6);
  private DigitalInput m_button = new DigitalInput(0);
  private Trigger m_coastModeButton = new Trigger(m_button::get).negate();
  @Log
  SendableChooser<AutoRoutine> m_autoSelector = new SendableChooser<>();
  private boolean m_setupDone = false;

  public RobotContainer(Consumer<Runnable> addPeriodic) {
    if (true || RobotBase.isSimulation()) {
      PhotonCamera.setVersionCheckEnabled(false);
    }
    m_shooterFeederS = new ShooterFeederS();
    m_shooterPivotS = new ShooterPivotS();
    m_shooterWheelsS = new ShooterWheelsS();
    m_midtakeS = new MidtakeS();
    m_intakePivotS = new IntakePivotS();
    m_intakeRollerS = new IntakeRollerS();
    m_lightStripS = LightStripS.getInstance();
    m_leftClimberS = new ClimberS(true);
    m_rightClimberS = new ClimberS(false);
    /* Changed the name to match the document name change. */
    m_ampPivotS = new CTREAmpPivotS();
    m_ampRollerS = new AmpRollerS();
    // Mechanism2d setup
    RobotVisualizer.setupVisualizer();
    RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_PIVOT);
    RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_TEST_PIVOT);
    RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_GOAL_PIVOT);
    RobotVisualizer.addMidtake(m_shooterFeederS.FEEDER_ROLLER);
    RobotVisualizer.addMidtake(m_midtakeS.MIDTAKE_ROLLER);
    
    RobotVisualizer.addAmpPivot(m_ampPivotS.AMP_PIVOT);
    m_intakePivotS.INTAKE_BEND.append(m_intakeRollerS.INTAKE_ROLLER);
    RobotVisualizer.addIntake(m_intakePivotS.INTAKE_PIVOT);
    // //m_climberS.TRAP_PIVOT_BASE.append(m_trapPivotS.TRAP_PIVOT);
    RobotVisualizer.addClimber(m_leftClimberS.ELEVATOR);
    RobotVisualizer.addClimber(m_rightClimberS.ELEVATOR);
    m_drivebaseS = TunerConstants.DriveTrain;
    m_noteCamera = new BlobDetectionCamera(addPeriodic, m_field.getObject("note"));
    LightStripS.setNoteAngles(m_noteCamera::getThreeTargetAngles);
    m_autos = new CommandGroups(
        m_drivebaseS,
        m_noteCamera,
        m_intakePivotS,
        m_intakeRollerS,
        m_midtakeS,
        m_ampPivotS,
        m_ampRollerS,
        m_shooterFeederS,
        m_shooterPivotS,
        m_shooterWheelsS,
        // m_climberS,
        m_lightStripS,
        m_driverController,
        (traj, starting)->{
          if (starting) {
            m_field.getObject("traj").setPoses(traj.getPoses());
          } else {
            m_field.getObject("traj").setPoses();
          }
        });
    configureDriverDisplay();
    configureButtonBindings();
    m_autos.addAutoRoutines(m_autoSelector);
    m_autoSelector.onChange(this::updateAutoDisplay);
    SignalLogger.setPath("/media/sda1/");
    SignalLogger.start();
    Monologue.setupMonologue(this, "Robot", false, true);
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);
    SparkDevice.burnFlashInSync();
    Commands.sequence(waitSeconds(2), runOnce(() -> m_setupDone = true))
        .ignoringDisable(true)
        .schedule();
    DriverStation.reportWarning("Setup Done", false);
    // PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("pathplanner").setPoses(poses));
    // PathPlannerLogging.setLogTargetPoseCallback(pose -> m_field.getObject("ppTarget").setPose(pose));
    
  }

  private void updateAutoDisplay(AutoRoutine routine) {
    m_field.getObject("auto").setPoses(routine.getPoses());
    log("autoTime", routine.estTime());
  }

  @Log
  public boolean notAtMidline() {
    return m_autos.notAtMidline();
  }
  // aiming left of the speaker is positive
  public double sidewaysErrorToSpeaker() {
    var error = m_drivebaseS.getPoseHeading()
                .minus(
                    Pathing.speakerDirection(
                        m_drivebaseS.getPose(),
                        speaker()))
                .getRadians();
    var tangent = Math.tan(error);
    if (tangent == Double.NaN) {
      return Double.POSITIVE_INFINITY;
    }
    var sideways = tangent * distanceToSpeaker();
    return sideways;
  }
  public void configureDriverDisplay() {
    m_driverDisplay.setHasNoteSupplier(m_midtakeS.hasNote);
    m_driverDisplay.setInRangeSupplier(() -> {
      var dist = distanceToSpeaker();
      return dist > Interpolation.MIN_DISTANCE && dist < Interpolation.MAX_DISTANCE;
    });
    m_driverDisplay.setInAngleSupplier(
        () -> Math.abs(sidewaysErrorToSpeaker()) < Units.inchesToMeters(3)
    );
    m_driverDisplay
        .setInPivotSupplier(() -> Math.abs(m_shooterPivotS.getAngle() - pivotAngle()) < Units.degreesToRadians(0.5));
    m_driverDisplay.setInSpeedSupplier(m_shooterWheelsS.atGoal);
    m_driverDisplay.setSeeNoteSupplier(m_drivebaseS.m_vision::hasTarget);
    m_driverDisplay.setIntakeHomedSupplier(m_intakePivotS::hasHomed);
  }

  public Translation2d speaker() {
    return m_autos.speaker();
  }
  
  public double xDistToSpeaker() {
    return Math.abs(m_drivebaseS.getPose().getTranslation().getX() - speaker().getX());
  }

  @Log
  public double directionToSpeaker() {
    return m_autos.directionToSpeaker();
  }

  @Log
  public double distanceToSpeaker() {
    return m_autos.distanceToSpeaker();
  }

  @Log
  public double pivotAngle() {
    return m_autos.pivotAngle();
  }
  public Command spinDistance(DoubleSupplier distance) {
    return m_shooterWheelsS.spinC(()->Interpolation.LEFT_MAP.get(distance.getAsDouble()),
      ()->Interpolation.RIGHT_MAP.get(distance.getAsDouble()) );
  }

  public Command faceSpeaker() {
    return m_drivebaseS.manualFieldHeadingDriveC(m_fwdXAxis, m_fwdYAxis,
        this::directionToSpeaker,
        () -> Pathing.aimingFFVelocity(
            m_drivebaseS.getPose(),
            m_drivebaseS.getFieldRelativeLinearSpeedsMPS(), NomadMathUtil.mirrorTranslation(
                Constants.Poses.SPEAKER,
                AllianceWrapper.getAlliance())))
        .alongWith(
        run(()->{
          var error  = m_drivebaseS.getPoseHeading().minus(new Rotation2d(this.directionToSpeaker())).getRadians();
          if (error >= Units.degreesToRadians(1)) {
            // drivetrain is too far ccw, light the left third of the bar
            LightStripS.getInstance().requestState(States.LeftThird);
          } else if (error <= Units.degreesToRadians(-1)) {
            LightStripS.getInstance().requestState(States.RightThird);
          } else {
            LightStripS.getInstance().requestState(States.CenterThird);
          }
        })        
        );
  }

  public Command faceNote() {
    return m_autos.faceNoteC(m_fwdXAxis, m_fwdYAxis,
                rumble -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumble));
  }
  public void configureButtonBindings() {
    InputAxis leftClimberStick = new InputAxis("LClimb", m_operatorController::getLeftY).withDeadband(0.2);
        InputAxis rightClimberStick = new InputAxis("RClimb", m_operatorController::getRightY).withDeadband(0.2);
    m_leftClimberS.isRaised.or(m_rightClimberS.isRaised).whileTrue(m_lightStripS.stateC(()-> States.Climbing));
    m_drivebaseS.setDefaultCommand(m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis));
    m_shooterPivotS.setDefaultCommand(
        m_shooterPivotS.rotateWithVelocity(
            this::pivotAngle,
            () -> 0
        // ()-> Interpolation.dThetadX(distanceToSpeaker()) *
        // -Pathing.velocityTorwardsSpeaker(
        // m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
        // speaker())
        ));
    //#region driver controller

    // button bindings for stage orientation
    // m_driverController.b().whileTrue(m_drivebaseS.manualHeadingDriveC(m_fwdXAxis, m_fwdYAxis, ()-> Math.PI/3.0));
    // m_driverController.x().whileTrue(m_drivebaseS.manualHeadingDriveC(m_fwdXAxis, m_fwdYAxis, ()-> -Math.PI/3.0));
    // m_driverController.y().whileTrue(m_drivebaseS.manualHeadingDriveC(m_fwdXAxis, m_fwdYAxis, ()-> Math.PI));

    // face note
    m_driverController.a().whileTrue(faceNote());
    m_driverController.y().whileTrue(faceSpeaker());

    // intaking
    m_driverController.leftBumper().onTrue(m_autos.retractStopIntake());
    m_driverController.rightBumper().onTrue(m_autos.deployRunIntake(m_driverController.rightBumper()));

    // face amp
    m_driverController.leftTrigger().whileTrue(m_autos.driveToPreAmp());
    // face speaker
    m_driverController.rightTrigger().whileTrue(faceSpeaker());

    /* Switched the wording to line up with the name of this system for the new motor */
    m_driverController.back().or(m_operatorController.back()).whileTrue(
      parallel(
        m_intakePivotS.resetToRetractedC(),
        m_ampPivotS.resetToRetractedC(),
        runOnce(m_shooterPivotS::resetAngleUp).ignoringDisable(true)
      ));
    m_driverController.start().onTrue(runOnce(m_shooterPivotS::resetAngleDown).ignoringDisable(true));
    m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());

    //#endregion

    m_coastModeButton.whileTrue(m_shooterPivotS.coast()).whileTrue(m_intakePivotS.coast());

    //#region operator controller start

    // RT: shoot
    // LT: spinup 
    // A: lock drive rotation for driveby
    // B: intake spit
    // X: midtake spit
    // Y: midtake move in
    // LB: spin for amp
    // RB: spin for passing
    // sticks: climb
    // 

/* Removed the code from the old motor */

     m_operatorController.b().whileTrue(m_intakeRollerS.outtakeC());
     //intake spit out
     m_operatorController.x().whileTrue(
      parallel(
      m_midtakeS.runVoltage(()-> (-0.6995 * 2),()-> (-0.6995 * 2)),
      m_shooterFeederS.runVoltageC(()-> -0.6995),
      m_intakeRollerS.runVoltageC(()->0.6995*2)));
     // intake move 
     m_operatorController.y().whileTrue(
      parallel(
      m_midtakeS.runVoltage(()-> 0.6995 * 2,()-> 0.6995 * 2),
      m_shooterFeederS.runVoltageC(()-> 0.6995),
      m_intakeRollerS.runVoltageC(()->-0.6995*2)
     ));
    //  // spinup for amp
    //  ampMode.whileTrue(
    //   parallel(m_shooterWheelsS.spinC(()->5000, ()->5000),
    //   m_shooterPivotS.rotateToAngle(()->Interpolation.AMP_PIVOT),
    //   m_bounceBarS.upC()
    //   )
    // );
    //m_operatorController.start().onTrue(m_bounceBarS.downC().withTimeout(1));
    //  // spinup for passing


    /* Re-added the code identically, with the exception of line 393 where .withTimeout(1.5)
    was added so that the scoring command doesn't run forever. 
    Also, we no longer need PID to hold the horizontal position. */

    // amp handoff
    Trigger ampIntake = m_operatorController.rightBumper();
    Trigger ampScore = m_operatorController.a();

    ampScore.onTrue(
      sequence(
        deadline(
          sequence(
            waitSeconds(0.1),
            waitUntil(m_ampPivotS.onTarget).withTimeout(1.5),
            m_ampRollerS.outtakeC().withTimeout(0.5),
            m_ampRollerS.stopC()
          ),
          m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.SCORE_ANGLE)
        ),
        m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.CW_LIMIT).withTimeout(1.5)
      ));
    ampIntake.whileTrue(
      sequence(
        parallel(
          /* Changed the name to match the document name change.
          Also changed the timeout time and fine-tuned the handoff position with the subtraction from CCW limit. */
        m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.CCW_LIMIT-Units.degreesToRadians(4))
        ).until(m_ampPivotS.onTarget).withTimeout(1.5),
        parallel(
          m_shooterPivotS.rotateToAngle(()->ShooterPivotS.Constants.CCW_LIMIT - Units.degreesToRadians(4)),
          /* A slight change to the RPM */
          m_shooterWheelsS.spinC(()->3000, ()->3000),
          m_shooterFeederS.runVoltageC(()->6),
          sequence(
            waitSeconds(0.1),
            waitUntil(m_ampPivotS.onTarget).withTimeout(4),
            /* A change in voltage because of the new motors */
            m_midtakeS.runVoltage(()->0.6995*6, ()->0.6995*6)
          ),
          m_ampRollerS.intakeC(),
          /* Changed the name to match the document name change. */
          m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.CCW_LIMIT)
        ).until(m_ampRollerS.receiveNote),
        parallel(
          new ScheduleCommand(
            /* Added timeouts and a better sequence in order to rotate to angles more accurately. */
            m_ampRollerS.intakeC().withTimeout(0.4)
          ),
          new ScheduleCommand(
            m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.CW_LIMIT).withTimeout(1.5)
          ),
          new ScheduleCommand(
            m_intakeRollerS.outtakeC().withTimeout(1)
          ),
          new ScheduleCommand(
            sequence(
              m_intakePivotS.rotateToAngle(()->IntakePivotS.Constants.RETRACTED - Units.degreesToRadians(10)).withTimeout(1),
              new ScheduleCommand(m_intakePivotS.retract().withTimeout(1))
            )
            
          ),
          new ScheduleCommand(m_shooterWheelsS.spinC(()->2000, ()->2000).withTimeout(1))
        )

      )
    );
    //   spinDistance(this::xDistToSpeaker),
    //   m_shooterPivotS.rotateWithVelocity(
    //         ()->Interpolation.PIVOT_MAP.get(xDistToSpeaker()),
    //         () -> 0)

    //  ));

    /* Deleted code that controls the old motor to replace below with code that is more appropriate for the new motor. */

     m_operatorController.rightTrigger().whileTrue(m_midtakeS.runVoltage(()->10.5, ()->10.5).alongWith(m_shooterFeederS.runVoltageC(()->12)));
     m_operatorController.leftTrigger().whileTrue(
      spinDistance(this::distanceToSpeaker).alongWith(
      m_shooterPivotS.rotateWithVelocity(
            this::pivotAngle,
            () -> 0)
     ));
    //m_operatorController.start().onTrue(runOnce(m_drivebaseS.m_vision::captureImages).ignoringDisable(true));
        // m_leftClimberS.setDefaultCommand(m_leftClimberS.runVoltage(()->-12* leftClimberStick.getAsDouble()));
        // m_rightClimberS.setDefaultCommand(m_rightClimberS.runVoltage(()->-12* rightClimberStick.getAsDouble()));

    m_ampPivotS.setDefaultCommand(m_ampPivotS.runVoltage(()->2*rightClimberStick.getAsDouble()));
    m_operatorController.povLeft().whileTrue(m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.CW_LIMIT));
    m_operatorController.povUp().whileTrue(m_ampPivotS.rotateToAngle(()->Math.PI/2));
    /* Changed the name to match the document name change. */
    m_operatorController.povRight().whileTrue(m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.SCORE_ANGLE));
    m_operatorController.povDown().whileTrue(m_ampPivotS.rotateToAngle(()->CTREAmpPivotS.Constants.CCW_LIMIT));
    //#endregion
  }

  public Command driveIntakeRelativePOV() {
    return m_drivebaseS.run(() -> {
      double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
      double adjustSpeed = Units.feetToMeters(6); // m/s
      m_drivebaseS.drive(
          new ChassisSpeeds(
              Math.cos(pov) * adjustSpeed,
              Math.sin(pov) * adjustSpeed,
              m_rotAxis.getAsDouble() * DriveConstants.MAX_TURN_SPEED));
    });
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.getSelected().cmd();
  }

  Pose3d origin = ZERO_POSE3D;
  Transform3d inBotNote = new Transform3d(0.1, 0, 0.3, new Rotation3d());
  /**
   * runs after subsystem periodics and after commands
   * */
  public void periodic() {
    Pose3d startPose = m_midtakeS.hasNote() ? new Pose3d(m_drivebaseS.getPose())
        .transformBy(inBotNote) : origin;
    log("notePose", startPose);
    if (DriverStation.isDisabled()) {
      if (m_setupDone) {
        LightStripS.getInstance().requestState(States.SetupDone);
      } else {
        LightStripS.getInstance().requestState(States.Disabled);
      }
    }
    TimingTracer.update();
    loopTime = loopTimeAverage.calculate(TimingTracer.getLoopTime());
    // /* Trace the loop duration and plot to shuffleboard */
    LightStripS.getInstance().periodic();
    updateFields();
    Monologue.setFileOnly(Robot.isReal() && DriverStation.isFMSAttached());
    var beforeLog = Timer.getFPGATimestamp();
    Monologue.updateAll();
    var afterLog = Timer.getFPGATimestamp();
    log("mlUpdate", (afterLog - beforeLog));
    m_driverDisplay.update();
  }

  StructArrayPublisher<Pose3d> notePosePub = NetworkTableInstance.getDefault().getStructArrayTopic("Robot/Note3d",Pose3d.struct).publish();
  Transform3d noteTransform = new Transform3d(0, 0, Units.inchesToMeters(1), new Rotation3d());
  public void updateFields() {
    m_drivebaseS.drawRobotOnField(m_field);
    m_driverField.getRobotObject().setPose(m_drivebaseS.getPose());
    List<Pose2d> poses = m_noteCamera.getTargets((time)->Optional.of(m_drivebaseS.getPose()));
    m_field.getObject("note").setPoses(poses);
    List<Pose3d> p3ds = poses.stream().map(p2->new Pose3d(p2).transformBy(noteTransform)).collect(Collectors.toList());
    notePosePub.accept(p3ds.toArray(new Pose3d[p3ds.size()]));
    //m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());

  }

  public void onEnabled() {
    m_drivebaseS.m_vision.captureImages();
  }

  public void onDisabled() {
    m_drivebaseS.m_vision.captureImages();
  }

  public Command shootVis() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
            () -> {
              final Pose3d startPose = new Pose3d(m_drivebaseS.getPose())
                  .transformBy(new Transform3d(0, 0, 0.3, new Rotation3d(0, -m_shooterPivotS.getAngle(), 0)));
              final boolean isRed = DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get().equals(Alliance.Red);
              final Pose3d endPose = new Pose3d(speaker().getX(), speaker().getY(), 2.1, startPose.getRotation());

              final double duration = startPose.getTranslation().getDistance(endPose.getTranslation()) / 3;
              final Timer timer = new Timer();
              timer.start();
              return Commands.run(
                  () -> {

                    log(
                        "NoteVisualizer",
                        new Pose3d[] {
                            startPose.interpolate(endPose, timer.get() / duration)
                        });
                  })
                  .until(() -> timer.hasElapsed(duration))
                  .finallyDo(
                      () -> {
                        log("NoteVisualizer", new Pose3d[] {});
                      });
            },
            Set.of())
            .ignoringDisable(true));
  }
}
