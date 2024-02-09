// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.midtake;

import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sparkmax.SparkDevice;

public class MidtakeS extends SubsystemBase {
  public class Constants {
    public static final int FRONT_CAN_ID = 30;
    public static final int BACK_CAN_ID = 31;
    public static final int CURRENT_LIMIT = 15;
    public static final double OUT_VOLTAGE = 6;
    public static final double IN_VOLTAGE = -6;
    // public static final Consumer<SparkBaseConfig> config = c=>{
    //   c.
    // }
  }
  private CANSparkMax m_front;
  private CANSparkMax m_back;

      public final MechanismLigament2d MIDTAKE_ROLLER = new MechanismLigament2d(
    "midtake-roller", Units.inchesToMeters(1), 0, 4, new Color8Bit(255, 255, 255));
  /** Creates a new IntakeRollerS. */
  public MidtakeS() {
    m_front = SparkDevice.getSparkMax(Constants.FRONT_CAN_ID);
    m_back = SparkDevice.getSparkMax(Constants.BACK_CAN_ID);
    m_front.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    m_front.setIdleMode(IdleMode.kBrake);
    m_back.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    m_back.setIdleMode(IdleMode.kBrake);
    setDefaultCommand(stopC());
  }

  @Override
  public void periodic() {
      MIDTAKE_ROLLER.setAngle(MIDTAKE_ROLLER.getAngle() + m_front.getAppliedOutput());
  }

  /**sets motor to outtake */
  public void feed () {
    m_front.setVoltage(Constants.OUT_VOLTAGE);
    m_back.setVoltage(Constants.OUT_VOLTAGE);

  }
  /**sets motor to intake */
  public void intake () {
    m_front.setVoltage(Constants.IN_VOLTAGE);
    m_back.setVoltage(Constants.IN_VOLTAGE);
  }
  /**stops the intake motor */
  public void stop() {
    m_front.setVoltage(0);
    m_back.setVoltage(0);
  }
  /**returns the command of the outtake */
  public Command outtakeC() {
    return run(this::feed);
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
