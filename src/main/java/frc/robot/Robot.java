package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AllianceWrapper;
import monologue.Monologue;

public class Robot extends TimedRobot {

  private static boolean isSimulation = false;
  
  private RobotContainer robotContainer;

  private Command autonomousCommand;

  @Override
  public void robotInit() {

    Robot.isSimulation = RobotBase.isSimulation();
    DriverStation.silenceJoystickConnectionWarning(true);

    LiveWindow.disableAllTelemetry();
    robotContainer = new RobotContainer((fn) -> this.addPeriodic(fn, kDefaultPeriod));
    // DataLogManager.logNetworkTables(true);
    addPeriodic(
        () -> {
          AllianceWrapper.setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
        },
        0.5);
    System.gc();
  }

  @Override
  public void robotPeriodic() {
    var beforeLog = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    var afterLog = Timer.getFPGATimestamp();
    robotContainer.log("commandsRun", (afterLog-beforeLog));
    
    robotContainer.periodic();
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void autonomousInit() {
    AllianceWrapper.setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
    robotContainer.onEnabled();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) autonomousCommand.schedule();
  }

  @Override
  public void teleopInit() {

    AllianceWrapper.setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
    robotContainer.onEnabled();
    if (autonomousCommand != null) autonomousCommand.cancel();
  }

  @Override
  public void disabledInit() {
    robotContainer.onDisabled();
    System.gc();
  }

  @Override
  public void disabledPeriodic() {}

  public static boolean isSimulation() {
    return isSimulation;
  }

  public static boolean isReal() {
    return !isSimulation;
  }
  
}
