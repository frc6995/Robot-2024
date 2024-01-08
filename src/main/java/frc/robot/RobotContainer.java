package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.util.InputAxis;
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkDevice;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  /** Establishes the controls and subsystems of the robot */
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final DrivebaseS m_drivebaseS;

   private double loopTime = 0;
  private LinearFilter loopTimeAverage = LinearFilter.movingAverage(1);

   private final Field2d m_field = new Field2d();

  private final Autos m_autos;

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

  
  private double getFwdAxis() {
    return m_fwdXAxis.getAsDouble();
  }

  
  private double getSideAxis() {
    return m_fwdYAxis.getAsDouble();
  }

  
  private double getRotAxis() {
    return m_rotAxis.getAsDouble();
  }

  public RobotContainer(Consumer<Runnable> addPeriodic) {
    if (RobotBase.isSimulation()) {
      PhotonCamera.setVersionCheckEnabled(false);
    }
    Timer.delay(0.1);
    m_drivebaseS =
        new DrivebaseS(
            addPeriodic,
            (name, traj) -> {
              m_field
                  .getObject(name)
                  .setPoses(
                      traj.getStates().stream()
                          .map(
                              (Function<State, Pose2d>)
                                  (State state) -> {
                                    return new Pose2d(
                                        state.positionMeters, state.targetHolonomicRotation);
                                  })
                          .collect(Collectors.toList()));
            });

    // Delay to let the motor configuration finish
    Timer.delay(0.1);

    m_autos = new Autos(m_drivebaseS);
    configureButtonBindings();
    addAutoRoutines();

    SmartDashboard.putData(m_autoSelector);
    // AutoLog.setupLogging(this, "Robot", true);
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);
    // Delay either side of burning flash on all spark maxes (this)
    Timer.delay(0.3);
    SparkDevice.burnFlashInSync();
    Timer.delay(0.2);
    Commands.sequence(waitSeconds(4), runOnce(() -> m_setupDone = true))
        .ignoringDisable(true)
        .schedule();
    DriverStation.reportWarning("Setup Done", false);
  }

  public void configureButtonBindings() {
    m_drivebaseS.setDefaultCommand(m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis));
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
    //AutoLog.update();
  }

  public void updateFields() {
    m_drivebaseS.drawRobotOnField(m_field);
    m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
  }

  public void onEnabled() {
    m_drivebaseS.resetRelativeRotationEncoders();
  }

  public void onDisabled() {}
}
