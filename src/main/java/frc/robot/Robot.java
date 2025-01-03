package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.Watchdog;
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
          if (DriverStation.isDisabled()) {
          AllianceWrapper.setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
          }

        },
        10);
    //addPeriodic(FaultLogger::update, 1);
    System.gc();
  }

  @Override
  public void robotPeriodic() {
    var beforeLog = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    
    
    robotContainer.periodic();
    //NetworkTableInstance.getDefault().flush();
    var afterLog = Timer.getFPGATimestamp();
    robotContainer.log("commandsRun", (afterLog-beforeLog));
  }

  @Override
  public void autonomousInit() {
    AllianceWrapper.setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
    robotContainer.onEnabled();
  }

  @Override
  public void teleopInit() {

    AllianceWrapper.setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
    robotContainer.onEnabled();
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
