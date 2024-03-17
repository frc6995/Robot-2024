// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberS.Constants.LOWER_LIMIT;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

/**
 * Convention:
 * angle is measured as the robot facing right, angles increase ccw
 * shooter pointing forward horizontal is 0
 * Shooter points toward the back of the robot so endstops are about 100-170 degrees
 * 
 */
public class ClimberS extends SubsystemBase implements Logged {
  public final Trigger isRaised;
  /**
   * IO class for interacting with motor.
   */
  private ClimberIO m_io;
  /**
   * The TrapezoidProfile for calculating smooth movement
   */
  private TrapezoidProfile m_profile;
  /**
   * The setpoint to be tracking at the moment.
   */
  private State m_setpoint = new State();
  /**
   * The end goal of the profile (our eventual desired position)
   */
  private State m_desiredState = new State();

  /**
   * A feedforward to assist the motor in keeping up with the profile.
   * 
   */
  public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    Constants.K_S, Constants.K_V, Constants.K_A);
  /**
   * For visualization.
   */
  public final MechanismLigament2d ELEVATOR; 
  // public final MechanismLigament2d TRAP_PIVOT_BASE = new MechanismLigament2d(
  //   "trap-pivot-base", 0, -90, 0, new Color8Bit());
  /** Creates a new ShooterPivotS. */
  public ClimberS(boolean isLeft) {
    // Create the IO class.
    if (Robot.isSimulation()) {
      m_io = new SimClimberIO(isLeft);
    }
    else {
      m_io = new RealClimberIO(isLeft);
    }
    m_profile = new TrapezoidProfile(Constants.CONSTRAINTS);
    if (isLeft){
      ELEVATOR= new MechanismLigament2d(
      "leftClimber", LOWER_LIMIT, 90, 12, new Color8Bit(235, 137, 52));
    } else {
            ELEVATOR= new MechanismLigament2d(
      "rightClimber", LOWER_LIMIT, 90, 6, new Color8Bit(235, 0, 0));
    }
    isRaised = new Trigger(()-> getLength() > LOWER_LIMIT + 0.01);
    setDefaultCommand(runVoltage(()->0));
    //ELEVATOR.append(TRAP_PIVOT_BASE);
    
  }
  @Log.NT public double getGoal() {return m_desiredState.position;}
  @Log.NT public double getGoalVelocity() {return m_desiredState.velocity;}
  @Log.NT public double getLength() {return m_io.getLength();}
  @Log.NT public double getPidVolts() {return m_io.getPidVolts();}
  @Log.NT public double getVolts() {return m_io.getVolts();}
  public void periodic() {
    // Update our visualization
    ELEVATOR.setLength(m_io.getLength());

    
    if (DriverStation.isEnabled()) {
      // If enabled, calculate the next step in the profile from our previous setpoint
      // to our desired state
      // m_setpoint = m_profile.calculate(0.02, m_setpoint, m_desiredState);
      // // log that information
      // log("setpointVelocity", m_setpoint.velocity);
      // log("setpointPosition", m_setpoint.position);
      // // Calculate the feedforward. This is partly to counter gravity
      // double ffVolts = getGravityFF() + getVelocityFF();

      // m_io.setPIDFF(m_setpoint.position, ffVolts);
    } else {
      // If disabled, continuously update setpoint and goal to avoid
      // sudden movement on re-enable.
      m_desiredState.velocity = 0;
      m_desiredState.position = m_io.getLength();
      m_setpoint.velocity = m_desiredState.velocity;
      m_setpoint.position = m_desiredState.position;
      m_io.setVolts(0);
    }
    m_io.periodic();
  }

  public void setLength(double length) {
    m_desiredState.position = length;
  }

  public Command runVoltage(DoubleSupplier voltage) {
    return run(()->m_io.setVolts(voltage.getAsDouble()));
  }

  public Command moveToLengthC(DoubleSupplier lengthSupplier) {
    return run(()->setLength(lengthSupplier.getAsDouble()));
  }

  /**
   * Calculates the voltage required to hold the pivot in its current position,
   * countering gravity.
   */
  @Log.NT
  public double getGravityFF() {
    return Constants.K_G;
  }

  /**
   * Calculates the voltage required to hold the pivot in its current position,
   * countering gravity.
   */
  @Log.NT
  public double getVelocityFF() {
    return m_feedforward.calculate(m_setpoint.velocity);
  }

  public class Constants {
    
    public static final double UPPER_LIMIT = Units.inchesToMeters(38);
    /**
     * The distance between trap pivot axle and bottom of climber when retracted
     */
    public static final double LOWER_LIMIT = Units.inchesToMeters(24);
    public static final int LEFT_CAN_ID = 51;
    public static final int RIGHT_CAN_ID = 52;
    /**
     * Also equivalent to motor radians per pivot radian
     */
    public static final double MOTOR_ROTATIONS_PER_METER = 25.0/(Math.PI*Units.inchesToMeters(1.875));
    public static final double K_G = 0;
    public static final double K_S = 0;

    public static final double K_V = 6.3;
    public static final double K_A = 0.00001;
    /**
     * radians per second, rad/s^2
     */
    public static final Constraints CONSTRAINTS = new Constraints(
      1, 2);
  }

}
