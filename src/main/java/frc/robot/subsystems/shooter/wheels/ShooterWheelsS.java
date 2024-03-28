// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.wheels;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.sparkmax.SparkDevice;
import monologue.Logged;

/**The shooter wheels that launch the note */
public class ShooterWheelsS implements Logged {
  private ShooterRoller m_topRoller;
  private ShooterRoller m_bottomRoller;

  public final Trigger topAtGoal;
  public final Trigger bottomAtGoal;

  public final Trigger atGoal;

  public class Constants{
    public static final int TOP_CAN_ID = 35;
    public static final int BOTTOM_CAN_ID = 36;

  }
  /** Creates a new ShooterWheelsS. */
  public ShooterWheelsS() {
    m_topRoller = new ShooterRoller(Constants.TOP_CAN_ID,  true, 0.05, 12/(9000.0 / 60.0), 0.05, 0, "Top");// kA 0.016432/60.0
    topSysId = m_topRoller.m_idRoutine;
    m_bottomRoller = new ShooterRoller(Constants.BOTTOM_CAN_ID, true, 0, 12/(9000.0 / 60.0), 0.05, 0, "Bottom"); //kA 0.017026/60.0
    bottomSysId = m_bottomRoller.m_idRoutine;

    topAtGoal = m_topRoller.atGoal;
    bottomAtGoal = m_bottomRoller.atGoal;

    atGoal = topAtGoal.and(bottomAtGoal);
  }
  /**Stops the shooter motors */
  public Command stopC(){
    return parallel(
      m_topRoller.stopC(),
      m_bottomRoller.stopC()
    );
  }

  public Command spinC(DoubleSupplier topSpeed, DoubleSupplier bottomSpeed){
    return parallel(
      m_topRoller.spinC(topSpeed),
      m_bottomRoller.spinC(bottomSpeed)
    );
  }
  public Command spinVoltageC(DoubleSupplier topVolts, DoubleSupplier bottomVolts){
    return parallel(
      m_topRoller.voltsC(topVolts),
      m_bottomRoller.voltsC(bottomVolts)
    );
  }

  public final SysIdRoutine topSysId;
  public final SysIdRoutine bottomSysId;

  public Command dynamic(Direction direction) {
    return topSysId.dynamic(direction).alongWith(bottomSysId.dynamic(direction));
  }

  public Command quasistatic(Direction direction) {
    return topSysId.quasistatic(direction).alongWith(bottomSysId.quasistatic(direction));
  }
}
