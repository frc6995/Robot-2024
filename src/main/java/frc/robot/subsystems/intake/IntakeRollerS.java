// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sparkmax.SparkDevice;

public class IntakeRollerS extends SubsystemBase {
  public class Constants {
    public static final int LEADER_CAN_ID = 20;
    public static final int FOLLOWER_CAN_ID = 21;
    public static final int CURRENT_LIMIT = 40;
    public static final double OUT_VOLTAGE = 6;
    public static final double IN_VOLTAGE = -8;
  }
  private CANSparkMax m_leader;
  private CANSparkMax m_follower;

    public final MechanismLigament2d INTAKE_ROLLER = new MechanismLigament2d(
    "intake-roller", Units.inchesToMeters(1), 0, 4, new Color8Bit(255, 255, 255));

  /** Creates a new IntakeRollerS. */
  public IntakeRollerS() {
    m_leader = SparkDevice.getSparkMax(Constants.LEADER_CAN_ID);
    m_follower = SparkDevice.getSparkMax(Constants.FOLLOWER_CAN_ID);
    m_leader.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    m_follower.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    m_leader.setInverted(false);
    m_follower.follow(m_leader);
    m_leader.setIdleMode(IdleMode.kCoast);
    m_follower.setIdleMode(IdleMode.kCoast);

    setDefaultCommand(stopC());
  }

  @Override
  public void periodic() {
      INTAKE_ROLLER.setAngle(INTAKE_ROLLER.getAngle() + m_leader.getAppliedOutput());
  }

  /**sets motor to outtake */
  public void outtake () {
    m_leader.setVoltage(Constants.OUT_VOLTAGE);
  }
  /**sets motor to intake */
  public void intake () {
    m_leader.setVoltage(Constants.IN_VOLTAGE);
  }
  /**stops the intake motor */
  public void stop() {
    m_leader.setVoltage(0);
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
