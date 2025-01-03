// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.wheels;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import monologue.Annotations.Log;
import monologue.Logged;

/**The shooter wheels that launch the note */
public class ShooterWheelsS implements Logged {
  private ShooterRoller m_leftRoller;
  private ShooterRoller m_rightRoller;

  public final Trigger leftAtGoal;
  public final Trigger rightAtGoal;

  public final Trigger atGoal;

  public class Constants{
    public static final int LEFT_CAN_ID = 35;
    public static final int RIGHT_CAN_ID = 36;
    public static final double LEFT_KV = 0.08436335189936792;
    public static final double RIGHT_KV = 0.08436335189936792;
  }
  /** Creates a new ShooterWheelsS. */
  public ShooterWheelsS() {
    
    m_leftRoller = new ShooterRoller(Constants.LEFT_CAN_ID,  true, 0.05, Constants.LEFT_KV, 0.05, 0.01, "Left");// kA 0.016432/60.0
    leftSysId = m_leftRoller.m_idRoutine;
    m_rightRoller = new ShooterRoller(Constants.RIGHT_CAN_ID, false, 0.05, 12/(5800* (38.0 / 23.0) / 60.0), 0.05, 0.01, "Right"); //kA 0.017026/60.0
    rightSysId = m_rightRoller.m_idRoutine;

    leftAtGoal = m_leftRoller.atGoal;
    rightAtGoal = m_rightRoller.atGoal;

    atGoal = leftAtGoal.and(rightAtGoal);
  }
  @Log
  public double leftPercent() {
    return m_leftRoller.getVelocity() / m_leftRoller.getGoalVelocity();
  }
  @Log
  public double rightPercent() {
    return m_rightRoller.getVelocity() / m_rightRoller.getGoalVelocity();
  }
  /**Stops the shooter motors */
  public Command stopC(){
    return parallel(
      m_leftRoller.stopC(),
      m_rightRoller.stopC()
    );
  }
  public Command stopOnceC(){
    return stopC().until(()->true);
  }

  public Command spinC(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
    return parallel(
      m_leftRoller.spinC(leftSpeed),
      m_rightRoller.spinC(rightSpeed)
    );
  }
  public Command spinVoltageC(DoubleSupplier leftVolts, DoubleSupplier rightVolts){
    return parallel(
      m_leftRoller.voltsC(leftVolts),
      m_rightRoller.voltsC(rightVolts)
    );
  }

  public final SysIdRoutine leftSysId;
  public final SysIdRoutine rightSysId;

  public Command dynamic(Direction direction) {
    return leftSysId.dynamic(direction).alongWith(rightSysId.dynamic(direction));
  }

  public Command quasistatic(Direction direction) {
    return leftSysId.quasistatic(direction).alongWith(rightSysId.quasistatic(direction));
  }
}
