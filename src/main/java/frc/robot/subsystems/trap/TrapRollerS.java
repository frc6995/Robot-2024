// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trap;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sparkmax.SparkDevice;

public class TrapRollerS extends SubsystemBase {
  public class Constants {
    public static final int CAN_ID = 54;
    public static final int CURRENT_LIMIT = 15;
    public static final double OUT_VOLTAGE = 6;
    public static final double IN_VOLTAGE = -6;
  }
  private CANSparkFlex m_motor;

  /** Creates a new IntakeRollerS. */
  public TrapRollerS() {
    m_motor = SparkDevice.getSparkFlex(Constants.CAN_ID);
    m_motor.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    m_motor.setIdleMode(IdleMode.kCoast);
    setDefaultCommand(stopC());
  }

  /**sets motor to outtake */
  public void outtake () {
    m_motor.setVoltage(Constants.OUT_VOLTAGE);
  }
  /**sets motor to intake */
  public void intake () {
    m_motor.setVoltage(Constants.IN_VOLTAGE);
  }
  /**stops the intake motor */
  public void stop() {
    m_motor.setVoltage(0);
  }
  /**returns the command of the outtake */
  public Command outtakeC() {
    return run(this::outtake);
  }
  /**returns the command of the intake */
  public Command intakeC() {
    return run(this::intake);
  }
  /**returns the command to stop the intake */
  public Command stopC(){
    return run(this::stop);
  }
}
