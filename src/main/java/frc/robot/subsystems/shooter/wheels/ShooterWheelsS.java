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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.sparkmax.SparkDevice;
import monologue.Logged;

/**The shooter wheels that launch the note */
public class ShooterWheelsS implements Logged {
  private ShooterRoller m_topRoller;
  private ShooterRoller m_bottomRoller;
  public class Constants{
    public static final int TOP_CAN_ID = 35;
    public static final int BOTTOM_CAN_ID = 36;
    public static final SimpleMotorFeedforward TOP_FF = new SimpleMotorFeedforward(0.1, 12.0/5743.0);
    public static final SimpleMotorFeedforward BOTTOM_FF = new SimpleMotorFeedforward(0.1, 12.0/5808.0);
  }
  /** Creates a new ShooterWheelsS. */
  public ShooterWheelsS() {
    m_topRoller = new ShooterRoller(Constants.TOP_CAN_ID,  true, Constants.TOP_FF, "Top");
    topSysId = m_topRoller.m_idRoutine;
    m_bottomRoller = new ShooterRoller(Constants.BOTTOM_CAN_ID, true, Constants.BOTTOM_FF, "Bottom");
    bottomSysId = m_bottomRoller.m_idRoutine;
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

  public final SysIdRoutine topSysId;
  public final SysIdRoutine bottomSysId;
}
