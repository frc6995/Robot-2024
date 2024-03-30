// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.midtake;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;
import monologue.Logged;
import monologue.Annotations.Log;

public class MidtakeS extends SubsystemBase implements Logged {
  public class Constants {
    public static final double TOF_NO_NOTE = 350;
    public static final int FRONT_CAN_ID = 32;
    public static final int BACK_CAN_ID = 30;
    public static final int CURRENT_LIMIT = 15;
    public static final double OUT_VOLTAGE = 2;
    public static final double IN_VOLTAGE = 6;
    public static final Consumer<SparkBaseConfig> config = c->{
      c.
        freeLimit(60)
        .stallLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .status6(32767)
        .status5(32767)
        .status4(32767)
        .status3(32767)
        .status2(32767)
        .status0(15)
      ;
    };
  }
  private CANSparkMax m_front;
  private CANSparkMax m_back;
  private TimeOfFlight m_tof;
  public final Trigger hasNote;
  public final Trigger recvNote;
  public final Trigger isRunning;

      public final MechanismLigament2d MIDTAKE_ROLLER = new MechanismLigament2d(
    "midtake-roller", Units.inchesToMeters(1), 0, 4, new Color8Bit(255, 255, 255));
  /** Creates a new IntakeRollerS. */
  public MidtakeS() {
    m_front = new SparkBaseConfig(Constants.config)
                .applyMax(
                  SparkDevice.getSparkMax(Constants.FRONT_CAN_ID), true
                );
    //m_front.setOpenLoopRampRate(0.25);
    m_back = new SparkBaseConfig(Constants.config)
                .status1(40)
                .status2(32767)
                //.status0(40)
                //.status1(32767)
                .applyMax(
                  SparkDevice.getSparkMax(Constants.BACK_CAN_ID), true
                );
    m_tof = new TimeOfFlight(32);
    m_tof.setRangingMode(RangingMode.Short, 24);
    m_tof.setRangeOfInterest(8, 8,12,12);
    hasNote = new Trigger(()->tofDistance() < 250);
    isRunning = new Trigger(()->getVolts() > 6).debounce(1);
    recvNote = new Trigger(()->m_front.getOutputCurrent() > 20);
    // m_front = SparkDevice.getSparkMax(Constants.FRONT_CAN_ID);
    // m_back = SparkDevice.getSparkMax(Constants.BACK_CAN_ID);
    // m_front.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    // m_front.setIdleMode(IdleMode.kBrake);
    // m_back.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    // m_back.setIdleMode(IdleMode.kBrake);
    setDefaultCommand(stopC());
  }

  @Override
  public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
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

  public void setVoltage(double frontVolts, double backVolts) {
    m_front.setVoltage(frontVolts);
    m_back.setVoltage(backVolts);
  }
  public void setVoltage(double volts) {
    setVoltage(volts, volts);
  }
  public Command runVoltage(DoubleSupplier frontVolts, DoubleSupplier backVolts) {
    return run(()->setVoltage(frontVolts.getAsDouble(), backVolts.getAsDouble()));
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
    public Command stopOnceC(){
    return runOnce(this::stop);
  }

  @Log
  public double tofDistance() {
    if (RobotBase.isSimulation()) {
      return Constants.TOF_NO_NOTE;
    }
    return m_tof.getRange();
  }

  @Log
  public boolean hasNote() {
    return hasNote.getAsBoolean();
  }
  public boolean recvNote() {
    return recvNote.getAsBoolean();
  }
  public boolean isRunning() {
    return isRunning.getAsBoolean();
  }
  public double getVolts() {
    return m_front.getAppliedOutput() * 12;
  }

  public double getBackVolts() {
    return m_back.getAppliedOutput() * 12;
  }

  public double getFrontCurrent() {
    return m_front.getOutputCurrent();
  }

  public double getBackCurrent() {
    return m_back.getOutputCurrent();
  }
}
