// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.wheels;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sparkmax.SparkDevice;

public class ShooterWheelsS extends SubsystemBase {
  private CANSparkFlex m_leftMotor;
  private CANSparkFlex m_rightMotor;
  public class Constants{
    public static final int LEFT_CAN_ID = 31;
    public static final int RIGHT_CAN_ID = 32;
  }
  /** Creates a new ShooterWheelsS. */
  public ShooterWheelsS() {
    m_leftMotor = SparkDevice.getSparkFlex(Constants.LEFT_CAN_ID);
    m_rightMotor = SparkDevice.getSparkFlex(Constants.RIGHT_CAN_ID);
    m_rightMotor.setInverted(true);
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);

    setDefaultCommand(stopC());
  }
  public void setVoltage(double voltage){
    m_leftMotor.setVoltage(voltage);
    m_rightMotor.setVoltage(voltage);
  }
  public Command stopC(){
    return run(()->setVoltage(0));
  }
  public Command spinC(double voltage){
    return run(()->setVoltage(voltage));
  }
  public Command spinC(DoubleSupplier voltage){
    return run(()->setVoltage(voltage.getAsDouble()));
  }
}
