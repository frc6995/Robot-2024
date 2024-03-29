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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
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
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkDevice;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.Set;
import java.util.function.Consumer;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer implements Logged {

  /** Establishes the controls and subsystems of the robot */
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_testingController = new CommandXboxController(5);
  private final CommandXboxController m_keypad = new CommandXboxController(2);
  private final DriverDisplay m_driverDisplay = new DriverDisplay();
  private final DrivebaseS m_drivebaseS;

  @Log.NT
  private final Mechanism2d MECH_VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
  private final ShooterFeederS m_shooterFeederS;
  private final ShooterPivotS m_shooterPivotS;
  private final ShooterWheelsS m_shooterWheelsS;
  private final IntakePivotS m_intakePivotS;
  private final IntakeRollerS m_intakeRollerS;
  private final MidtakeS m_midtakeS;
  private final ClimberS m_leftClimberS;
  private final ClimberS m_rightClimberS;
  private final BlobDetectionCamera m_noteCamera;
  private final LightStripS m_lightStripS;
  @Log.NT
  private double loopTime = 0;
  private LinearFilter loopTimeAverage = LinearFilter.movingAverage(1);
  @Log.NT
  private final Field2d m_field = new Field2d();
  @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
  private final Field2d m_driverField = new Field2d();

  private final CommandGroups m_autos;

  private InputAxis m_fwdXAxis = new InputAxis("Forward", m_driverController::getLeftY)
      .withDeadband(0.1)
      .withInvert(true)
      .withSlewRate(3)
      .withSquaring(true);
  private InputAxis m_fwdYAxis = new InputAxis("Strafe", m_driverController::getLeftX)
      .withDeadband(0.1)
      .withInvert(true)
      .withSlewRate(3)
      .withSquaring(true);
  private InputAxis m_rotAxis = new InputAxis("Rotate", m_driverController::getRightX)
      .withDeadband(0.2)
      .withInvert(true)
      .withSlewRate(1.33, -6);
  @Log
  SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

  private boolean m_setupDone = false;

  @Log.NT
  private double getFwdAxis() {
    return m_fwdXAxis.getAsDouble();
  }

  @Log.NT
  private double getSideAxis() {
    return m_fwdYAxis.getAsDouble();
  }

  @Log.NT
  private double getRotAxis() {
    return m_rotAxis.getAsDouble();
  }

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
    RobotVisualizer.setupVisualizer();
    RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_PIVOT);
    RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_TEST_PIVOT);
    RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_GOAL_PIVOT);
    RobotVisualizer.addMidtake(m_midtakeS.MIDTAKE_ROLLER);
    m_intakePivotS.INTAKE_BEND.append(m_intakeRollerS.INTAKE_ROLLER);
    RobotVisualizer.addIntake(m_intakePivotS.INTAKE_PIVOT);
    // //m_climberS.TRAP_PIVOT_BASE.append(m_trapPivotS.TRAP_PIVOT);
    RobotVisualizer.addClimber(m_leftClimberS.ELEVATOR);
    RobotVisualizer.addClimber(m_rightClimberS.ELEVATOR);
    m_drivebaseS = new DrivebaseS(
        addPeriodic,
        (name, poses) -> m_field.getObject(name).setPoses(poses));
    m_noteCamera = new BlobDetectionCamera(addPeriodic, m_field.getObject("note"));

    m_autos = new CommandGroups(
        m_drivebaseS,
        m_noteCamera,
        m_intakePivotS,
        m_intakeRollerS,
        m_midtakeS,
        m_shooterFeederS,
        m_shooterPivotS,
        m_shooterWheelsS,
        // m_climberS,
        m_lightStripS);
    configureDriverDisplay();
    configureButtonBindings();
    addAutoRoutines();

    SmartDashboard.putData(m_autoSelector);
    Monologue.setupMonologue(this, "Robot", false, true);
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);
    SparkDevice.burnFlashInSync();
    Commands.sequence(waitSeconds(4), runOnce(() -> m_setupDone = true))
        .ignoringDisable(true)
        .schedule();
    DriverStation.reportWarning("Setup Done", false);
    // m_shooterPivotS.setDefaultCommand(m_shooterPivotS.rotateWithVelocity(
    // this::pivotAngle,
    // ()-> Interpolation.dThetadX(distanceToSpeaker()) *
    // -Pathing.velocityTorwardsSpeaker(
    // m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
    // speaker())
    // )
    // );
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("pathplanner").setPoses(poses));
    PathPlannerLogging.setLogTargetPoseCallback(pose -> m_field.getObject("ppTarget").setPose(pose));
  }

  public void configureDriverDisplay() {
    m_driverDisplay.setHasNoteSupplier(m_midtakeS.hasNote);
    m_driverDisplay.setInRangeSupplier(() -> {
      var dist = distanceToSpeaker();
      return dist > Interpolation.MIN_DISTANCE && dist < Interpolation.MAX_DISTANCE;
    });
    m_driverDisplay.setInAngleSupplier(
        () -> Math.abs(
            m_drivebaseS.getPoseHeading()
                .minus(
                    Pathing.speakerDirection(
                        m_drivebaseS.getPose(),
                        speaker()))
                .getRadians()) < Units.degreesToRadians(0.5) // TODO set threshold based on dist from spkr
    );
    m_driverDisplay
        .setInPivotSupplier(() -> Math.abs(m_shooterPivotS.getAngle() - pivotAngle()) < Units.degreesToRadians(0.5));
    m_driverDisplay.setInSpeedSupplier(m_shooterWheelsS.atGoal);
    m_driverDisplay.setSeeNoteSupplier(m_noteCamera.hasTarget);
  }

  public Translation2d speaker() {
    return m_autos.speaker();
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

  public Command faceSpeaker() {
    return m_drivebaseS.manualFieldHeadingDriveC(m_fwdXAxis, m_fwdYAxis,
        this::directionToSpeaker,
        () -> Pathing.aimingFFVelocity(
            m_drivebaseS.getPose(),
            m_drivebaseS.getFieldRelativeLinearSpeedsMPS(), NomadMathUtil.mirrorTranslation(
                Constants.Poses.SPEAKER,
                AllianceWrapper.getAlliance())));
  }

  public Command faceNote() {
    return m_autos.faceNoteC(m_fwdXAxis, m_fwdYAxis,
                rumble -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumble));
  }
  public void configureButtonBindings() {
    m_drivebaseS.setDefaultCommand(m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis));
    m_driverController.a().whileTrue(faceSpeaker());
    m_driverController.rightTrigger().whileTrue(m_shooterWheelsS.spinVoltageC(()->10, ()->10));
    m_driverController.b().whileTrue(faceNote());
    m_driverController.rightBumper().onTrue(m_autos.deployRunIntake());

    m_driverController.x().whileTrue(m_intakeRollerS.outtakeC());
    m_driverController.leftBumper().onTrue(m_autos.retractStopIntake());// .whileTrue(m_intakePivotS.rotateToAngle(()->MathUtil.interpolate(IntakePivotS.Constants.CCW_LIMIT,
                                                               // IntakePivotS.Constants.CW_LIMIT,
                                                               // m_driverController.getRightTriggerAxis())));
    m_driverController.back().whileTrue(m_intakePivotS.resetToRetractedC());
    m_driverController.leftTrigger().whileTrue(m_midtakeS.intakeC().alongWith(m_shooterFeederS.feedC()));
    
    m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());
    // keypad only below this line
    m_keypad.button(1).whileTrue(m_shooterPivotS.runVoltage(() -> 0.1));
    m_keypad.button(4).whileTrue(m_shooterPivotS.runVoltage(() -> -0.7));
    m_keypad.button(5).whileTrue(m_shooterPivotS.rotateToAngle(() -> Units.degreesToRadians(180 - 25)));
    m_keypad.button(6).onTrue(
            m_shooterPivotS.rotateWithVelocity(
            this::pivotAngle,
            () -> 0));
    m_shooterPivotS.setDefaultCommand(
        m_shooterPivotS.rotateWithVelocity(
            this::pivotAngle,
            () -> 0
        // ()-> Interpolation.dThetadX(distanceToSpeaker()) *
        // -Pathing.velocityTorwardsSpeaker(
        // m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
        // speaker())
        ));
    //m_shooterPivotS.setDefaultCommand(m_shooterPivotS.hold());
    m_keypad.button(2).onTrue(
        runOnce(m_shooterPivotS::resetAngleDown).ignoringDisable(true));
    m_keypad.button(14).whileTrue(m_shooterWheelsS.spinC(() -> 6000, () -> 6000));
    m_keypad.button(12).whileTrue(m_shooterWheelsS.spinVoltageC(()->10, ()->10));
    m_keypad.button(11).whileTrue(m_shooterWheelsS.spinC(()->5700, ()->6000));
    m_keypad.button(13).whileTrue(m_shooterWheelsS.spinC(() -> 1500, () -> 1500));
    // m_testingController.back().onTrue(shootVis());
  }

  public Command driveIntakeRelativePOV() {
    return m_drivebaseS.run(() -> {
      double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
      double adjustSpeed = 0.75; // m/s
      m_drivebaseS.drive(
          new ChassisSpeeds(
              Math.cos(pov) * adjustSpeed,
              Math.sin(pov) * adjustSpeed,
              m_rotAxis.getAsDouble() * DriveConstants.MAX_TURN_SPEED));
    });
  }

  public void addAutoRoutines() {
    m_autoSelector.setDefaultOption("Do Nothing", none());
    m_autoSelector.addOption("W2", m_autos.centerWingNote(PathPlannerPath.fromChoreoTrajectory("W2")));
    m_autoSelector.addOption("W1", m_autos.centerWingNote(PathPlannerPath.fromChoreoTrajectory("W1")));
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.getSelected();
  }

  public void periodic() {
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
    var beforeLog = Timer.getFPGATimestamp();
    Monologue.updateAll();
    var afterLog = Timer.getFPGATimestamp();
    log("mlUpdate", (afterLog - beforeLog));
    m_driverDisplay.update();
  }

  public void updateFields() {
    m_drivebaseS.drawRobotOnField(m_field);
    m_driverField.getRobotObject().setPose(m_drivebaseS.getPose());
    m_field.getObject("note").setPoses(m_noteCamera.getTargets(m_drivebaseS::getOldPose));
    m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
  }

  public void onEnabled() {
    m_drivebaseS.resetRelativeRotationEncoders();
  }

  public void onDisabled() {

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
