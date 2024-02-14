package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.drive.DrivebaseS;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.intake.IntakeRollerS;
import frc.robot.subsystems.intake.pivot.IntakePivotS;
import frc.robot.subsystems.shooter.midtake.MidtakeS;
import frc.robot.subsystems.vision.BlobDetectionCamera;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkDevice;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

import java.util.function.Consumer;
import org.photonvision.PhotonCamera;

public class RobotContainer implements Logged {

  /** Establishes the controls and subsystems of the robot */
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final DrivebaseS m_drivebaseS;

  @Log.NT
  private final Mechanism2d MECH_VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
  // private final ShooterPivotS m_shooterPivotS;
  // private final ShooterWheelsS m_shooterWheelsS;
  private final IntakePivotS m_intakePivotS;
  private final IntakeRollerS m_intakeRollerS;
  private final MidtakeS m_midtakeS;
  // private final ClimberS m_climberS;
  private final BlobDetectionCamera m_noteCamera;
  private final LightStripS m_lightStripS;
  @Log.NT
  private double loopTime = 0;
  private LinearFilter loopTimeAverage = LinearFilter.movingAverage(1);
  @Log.NT
  private final Field2d m_field = new Field2d();

  private final CommandGroups m_autos;

  private InputAxis m_fwdXAxis =
      new InputAxis("Forward", m_driverController::getLeftY)
          .withDeadband(0.1)
          .withInvert(true)
          .withSlewRate(3)
          .withSquaring(true);
  private InputAxis m_fwdYAxis =
      new InputAxis("Strafe", m_driverController::getLeftX)
          .withDeadband(0.1)
          .withInvert(true)
          .withSlewRate(3)
          .withSquaring(true);
  private InputAxis m_rotAxis =
      new InputAxis("Rotate", m_driverController::getRightX)
          .withDeadband(0.2)
          .withInvert(true)
          .withSlewRate(1.33, -6);
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
    // m_shooterPivotS = new ShooterPivotS();
    // m_shooterWheelsS = new ShooterWheelsS();
   m_midtakeS = new MidtakeS();
    m_intakePivotS = new IntakePivotS();
    m_intakeRollerS = new IntakeRollerS();
    m_lightStripS = LightStripS.getInstance();
    // ///m_trapPivotS = new TrapPivotS();
    // m_climberS = new ClimberS();
    RobotVisualizer.setupVisualizer();
    // RobotVisualizer.addShooter(m_shooterPivotS.SHOOTER_PIVOT);
    // RobotVisualizer.addMidtake(m_midtakeS.MIDTAKE_ROLLER);
    m_intakePivotS.INTAKE_BEND.append(m_intakeRollerS.INTAKE_ROLLER);
    RobotVisualizer.addIntake(m_intakePivotS.INTAKE_PIVOT);
    // //m_climberS.TRAP_PIVOT_BASE.append(m_trapPivotS.TRAP_PIVOT);
    // RobotVisualizer.addClimber(m_climberS.ELEVATOR);
    Timer.delay(0.1);
    m_drivebaseS =
        new DrivebaseS(
            addPeriodic,
            (name, poses) -> m_field.getObject(name).setPoses(poses)
            );
    m_noteCamera = new BlobDetectionCamera(addPeriodic, m_field.getObject("note"));
    // Delay to let the motor configuration finish
    Timer.delay(0.1);

    m_autos = new CommandGroups(
      m_drivebaseS,
      m_noteCamera,
      m_intakePivotS,
      m_intakeRollerS,
       m_midtakeS,
      // m_shooterPivotS,
      // m_shooterWheelsS,
      //m_climberS,
      m_lightStripS);
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
  }

  public void configureButtonBindings() {
    m_drivebaseS.setDefaultCommand(m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis));
    m_driverController.rightTrigger().whileTrue(
      m_drivebaseS.manualFieldHeadingDriveC(m_fwdXAxis, m_fwdYAxis,
      ()->Pathing.speakerDirection(
        m_drivebaseS.getPose(),
        NomadMathUtil.mirrorTranslation(
          Constants.Poses.SPEAKER,
          AllianceWrapper.getAlliance())
        ).getRadians(),
      ()->Pathing.aimingFFVelocity(
        m_drivebaseS.getPose(),
        m_drivebaseS.getFieldRelativeLinearSpeedsMPS(), NomadMathUtil.mirrorTranslation(
          Constants.Poses.SPEAKER,
          AllianceWrapper.getAlliance()))));
    m_driverController.leftTrigger().whileTrue(
      m_drivebaseS.manualFieldHeadingDriveC(m_fwdXAxis, m_fwdYAxis,
      ()->Pathing.speakerDirection(
        m_drivebaseS.getPose(),
        NomadMathUtil.mirrorTranslation(
          Constants.Poses.SPEAKER,
          AllianceWrapper.getAlliance())
        ).getRadians(),
      ()->0));
    m_driverController.leftBumper().whileTrue(
      m_autos.faceNoteC(m_fwdXAxis, m_fwdYAxis));
    m_driverController.a().onTrue(m_autos.deployRunIntake());
    // m_driverController.a().and(
    //   new Trigger(m_noteCamera::hasTarget).negate())
    //   .whileTrue(
    //     run(()->m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.8)).ignoringDisable(true))
    //     .onFalse(
    //       runOnce(()->m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0)).ignoringDisable(true)        );
    m_driverController.x().whileTrue(m_intakeRollerS.outtakeC());
    m_driverController.b().whileTrue(m_intakePivotS.runVoltage(()->m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()));
    m_driverController.y().onTrue(m_autos.retractStopIntake());//.whileTrue(m_intakePivotS.rotateToAngle(()->MathUtil.interpolate(IntakePivotS.Constants.CCW_LIMIT, IntakePivotS.Constants.CW_LIMIT, m_driverController.getRightTriggerAxis())));
    m_driverController.back().whileTrue(m_intakePivotS.resetToRetractedC());
    m_driverController.start().whileTrue(m_midtakeS.intakeC());
    //m_driverController.leftBumper().whileTrue(m_autos.autoPickupC());
    // m_driverController.a().whileTrue(m_autos.driveToNote());
    // m_driverController.x().onTrue(m_shooterPivotS.run(()->
    // m_shooterPivotS.setAngle((ShooterPivotS.Constants.CW_LIMIT + ShooterPivotS.Constants.CCW_LIMIT) / 2.0)));
    // m_driverController.y()
    //   .whileTrue(
    //     sequence(
    //     deadline(
    //       m_autos.midtakeReceiveNote().asProxy(),
    //       m_autos.deployRunIntake()
    //     ),
    //     m_autos.retractStopIntake()
    //   )
    // );
    // m_driverController.b().onTrue(m_climberS.run(()->
    // m_climberS.setLength(ClimberS.Constants.UPPER_LIMIT)));
    // m_driverController.button(5).whileTrue(
    //   m_intakePivotS.m_idRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward)
    //   .until(()->m_intakePivotS.getAngle() > IntakePivotS.Constants.CCW_LIMIT - Units.degreesToRadians(5))
    // );
    // m_driverController.button(6).whileTrue(
    //   m_intakePivotS.m_idRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse)
    //   .until(()->m_intakePivotS.getAngle() < IntakePivotS.Constants.CW_LIMIT + Units.degreesToRadians(5))
    // );
    // m_driverController.button(7).whileTrue(
    //   m_intakePivotS.m_idRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward)
    //   .until(()->m_intakePivotS.getAngle() > IntakePivotS.Constants.CCW_LIMIT - Units.degreesToRadians(10))
    // );
    // m_driverController.button(8).whileTrue(
    //   m_intakePivotS.m_idRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse)
    //   .until(()->m_intakePivotS.getAngle() < IntakePivotS.Constants.CW_LIMIT + Units.degreesToRadians(10))
    // );
  }

  public void addAutoRoutines() {
    m_autoSelector.setDefaultOption("Do Nothing", none());
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
    log("mlUpdate", (afterLog-beforeLog));    
  }

  public void updateFields() {
    m_drivebaseS.drawRobotOnField(m_field);
    m_field.getObject("note").setPoses(m_noteCamera.getTargets(m_drivebaseS.getPose()));
    m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
  }

  public void onEnabled() {
    m_drivebaseS.resetRelativeRotationEncoders();
  }

  public void onDisabled() {}
}
