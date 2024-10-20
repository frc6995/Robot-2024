// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bounceBar;

import com.revrobotics.CANSparkMax;

import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FaultLogger;
import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;
import monologue.Logged;
import monologue.Annotations.Log;

public class BounceBarS extends SubsystemBase implements Logged {
  public class Constants {
    public static final int LEADER_CAN_ID = 41;
    public static final int CURRENT_LIMIT = 100;
    public static final double DOWN_VOLTAGE = -0.1;
    public static final double UP_VOLTAGE = 12;
        public static final Consumer<SparkBaseConfig> config = c->{
      c.
        freeLimit(40)
        .stallLimit(30)
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .status6(32767)
        .status5(32767)
        .status4(32767)
        .status3(32767)
        .status2(32767)
        .status0(15);
    };
  }
  private CANSparkMax m_leader;

    public final MechanismLigament2d INTAKE_ROLLER = new MechanismLigament2d(
    "intake-roller", Units.inchesToMeters(1), 0, 4, new Color8Bit(255, 255, 255));

  /** Creates a new IntakeRollerS. */
  public BounceBarS() {
    m_leader = new SparkBaseConfig(Constants.config)
    .applyMax(
      SparkDevice.getSparkMax(Constants.LEADER_CAN_ID), true
    );
    m_leader.setIdleMode(IdleMode.kCoast);
    // m_leader = SparkDevice.getSparkMax(Constants.LEADER_CAN_ID);
    // m_follower = SparkDevice.getSparkMax(Constants.FOLLOWER_CAN_ID);
    // m_leader.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    // m_follower.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    // m_leader.setInverted(false);
    // m_follower.follow(m_leader);
    // m_leader.setIdleMode(IdleMode.kCoast);
    // m_follower.setIdleMode(IdleMode.kCoast);

    setDefaultCommand(downC());
  }

  /**sets motor to outtake */
  public void down () {
    m_leader.setVoltage(Constants.DOWN_VOLTAGE);
  }
  /**sets motor to intake */
  public void up () {
    m_leader.setVoltage(Constants.UP_VOLTAGE);
  }
  /**stops the intake motor */
  public void stop() {
    m_leader.setVoltage(0);
  }
  /**returns the command of the outtake */
  public Command downC() {
    return run(this::down);
  }
  /**returns the command of the intake */
  public Command upC() {
    return run(this::up);
  }
  /**returns the command to stop the intake */
  public Command stopC(){
    return run(this::stop);
  }

  public double getCurrent() {
    return m_leader.getOutputCurrent();
  }
}
