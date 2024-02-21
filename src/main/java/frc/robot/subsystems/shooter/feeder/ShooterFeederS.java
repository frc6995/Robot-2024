// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.feeder;

import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;

public class ShooterFeederS extends SubsystemBase {
  public class Constants {
    public static final int CAN_ID = 33;
    public static final int CURRENT_LIMIT = 15;
    public static final double OUT_VOLTAGE = 12;
    public static final double THROUGH_VOLTAGE = 6;
    public static final Consumer<SparkBaseConfig> config = c->{
      c.freeLimit(CURRENT_LIMIT).idleMode(IdleMode.kCoast)
      .status6(32767)
      .status5(32767)
      .status4(32767)
      .status3(32767)
      .status2(32767)
      .status0(15);
    };
  }
  private CANSparkMax m_motor;

  /** Creates a new IntakeDirectorS. */
  public ShooterFeederS() {
    m_motor = new SparkBaseConfig(Constants.config).applyMax(
      SparkDevice.getSparkMax(Constants.CAN_ID), true);
    setDefaultCommand(stopC());
  }

  /**sets motor to outtake */
  public void feed () {
    m_motor.setVoltage(Constants.OUT_VOLTAGE);
  }
  /**sets motor to intake */
  public void backup () {
    m_motor.setVoltage(Constants.THROUGH_VOLTAGE);
  }
  /**stops the intake motor */
  public void stop() {
    m_motor.setVoltage(0);
  }
  /**returns the command of the outtake */
  public Command feedC() {
    return run(this::feed);
  }
  /**returns the command of the intake */
  public Command backupC() {
    return run(this::backup);
  }
  /**returns the command to stop the intake */
  public Command stopC(){
    return run(this::stop);
  }
}
