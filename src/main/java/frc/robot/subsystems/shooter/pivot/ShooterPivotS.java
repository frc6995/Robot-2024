// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import monologue.Logged;

/**
 * Convention:
 * angle is measured as the robot facing right, angles increase ccw
 * shooter pointing forward horizontal is 0
 * Shooter points toward the back of the robot so endstops are about 100-170 degrees
 * 
 */
public class ShooterPivotS extends SubsystemBase implements Logged {
  private ShooterPivotIO m_io;
  private TrapezoidProfile m_profile;
  private State m_currentState;
  private State m_desiredState;
  public final MechanismLigament2d SHOOTER_PIVOT = new MechanismLigament2d(
    "shooter", Constants.CG_DIST * 2, 0);
  /** Creates a new ShooterPivotS. */
  public ShooterPivotS() {
    if (Robot.isSimulation()) {
      m_io = new SimShooterPivotIO();
    }
    else {
      m_io = new RealShooterPivotIO();
    }
    m_profile = new TrapezoidProfile(Constants.CONSTRAINTS);
    m_desiredState = new State();
    m_currentState = new State();
  }

  public void periodic() {
    SHOOTER_PIVOT.setAngle(Units.radiansToDegrees(m_io.getAngle()));
    m_currentState.position = m_io.getAngle();
    m_currentState.velocity = 0; //TODO

    State setpoint = m_profile.calculate(0.02, m_currentState, m_desiredState);
    m_io.setPIDFF(setpoint.position, getGravityFF());
    m_io.periodic();
  }

  public void setAngle(double angle) {
    m_desiredState.position = angle;
  }

  public Command runVoltage(DoubleSupplier voltage) {
    return run(()->m_io.setVolts(voltage.getAsDouble()));
  }

  /**
   * Calculates the voltage required to hold the pivot in its current position,
   * countering gravity.
   */
  public double getGravityFF() {
    return Constants.K_G * Math.cos(m_io.getAngle());
  }
  public class Constants {
    public static final double CCW_LIMIT = Units.degreesToRadians(170);
    public static final double CW_LIMIT = Units.degreesToRadians(100);
    public static final int CAN_ID = 30;
    public static final double K_G = 0.2;
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 15;
    public static final double CG_DIST = Units.inchesToMeters(30);
    /**
     * radians per second, rad/s^2
     */
    public static final Constraints CONSTRAINTS = new Constraints(
      0.2, 0.2);
  }

}
