// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.shooter.pivot.ShooterPivotS.Constants.CCW_LIMIT;
import static frc.robot.subsystems.shooter.pivot.ShooterPivotS.Constants.CW_LIMIT;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

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
import edu.wpi.first.wpilibj2.command.Commands;
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
public class ShooterPivotS extends SubsystemBase implements Logged {
  /**
   * IO class for interacting with motor.
   */
  private ShooterPivotIO m_io;
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
  public final MechanismLigament2d SHOOTER_PIVOT = new MechanismLigament2d(
    "shooter", 8, Units.radiansToDegrees(CW_LIMIT), 4, new Color8Bit(235, 137, 52));

  public final MechanismLigament2d SHOOTER_TEST_PIVOT = new MechanismLigament2d(
    "shooter_test", 8, Units.radiansToDegrees(CW_LIMIT), 4, new Color8Bit(235, 137, 52));
    public final MechanismLigament2d SHOOTER_GOAL_PIVOT = new MechanismLigament2d(
    "shooter_goal", 4, Units.radiansToDegrees(CW_LIMIT), 4, new Color8Bit(235, 235, 235));
  /** Creates a new ShooterPivotS. */
  public ShooterPivotS() {
    // Create the IO class.
    if (Robot.isSimulation()) {
      m_io = new SimShooterPivotIO();
    }
    else {
      m_io = new RealShooterPivotIO();
    }
    m_profile = new TrapezoidProfile(Constants.CONSTRAINTS);
    resetAngleUp();
    setDefaultCommand(hold());
  }
  @Log public double getGoal() {return m_desiredState.position;}
  @Log public double getGoalVelocity() {return m_desiredState.velocity;}
  @Log public double getSetpoint() {return m_setpoint.position;}
  @Log public double getSetpointVelocity() {return m_setpoint.velocity;}
  @Log public double getAngle() {return m_io.getAngle();}
  @Log public double getVelocity() {return m_io.getVelocity();}
  @Log public double getPidVolts() {return m_io.getPidVolts();}
  @Log public double getVolts() {return m_io.getVolts();}
  @Log public double getCurrent() {return m_io.getCurrent();}
  public void resetProfile() {
    m_setpoint.position = getAngle();
  }
  public void resetAngleDown() {
    m_io.resetAngle(CCW_LIMIT);
  }
  public void resetAngleUp() {
    m_io.resetAngle(CW_LIMIT);
  }
  public void periodic() {
    // Update our visualization
    SHOOTER_PIVOT.setAngle(Units.radiansToDegrees(m_io.getAngle()));
    SHOOTER_GOAL_PIVOT.setAngle(Units.radiansToDegrees(m_desiredState.position));
    

    
    if (DriverStation.isEnabled()) {
      // If enabled, calculate the next step in the profile from our previous setpoint
      // to our desired state


    } else {
      // If disabled, continuously update setpoint and goal to avoid
      // sudden movement on re-enable.
      m_desiredState.velocity = 0;
      m_desiredState.position = m_io.getAngle();
      m_setpoint.velocity = m_desiredState.velocity;
      m_setpoint.position = m_desiredState.position;
      m_io.setVolts(0);
    }
    m_io.periodic();
  }

    public void setAngle(double angle) {
      setAngle(angle, 0);
    }
  public void setAngle(double angle, double velocity) {
    m_desiredState.position = angle;
    var nextSetpoint = m_profile.calculate(0.04, m_setpoint, m_desiredState);
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_desiredState);
    
    // log that information
    log("setpointVelocity", m_setpoint.velocity);
    log("setpointPosition", m_setpoint.position);
    log("dbCompVelocity", velocity);
    log("totalSetptVel", m_setpoint.velocity + velocity);
    var totalVelocity = m_setpoint.velocity + velocity;
    // Calculate the feedforward. This is partly to counter gravity
    double ffVolts = getGravityFF()+ m_feedforward.calculate(totalVelocity, nextSetpoint.velocity + velocity, 0.02);
    m_io.setPIDFF(m_setpoint.position, ffVolts);
  }


  public Command runVoltage(DoubleSupplier voltage) {
    return run(()->m_io.setVolts(voltage.getAsDouble()));
  }

  public Command rotateToAngle(DoubleSupplier angleSupplier) {
    return run(()->setAngle(angleSupplier.getAsDouble()));
  }
  public Command handoffAngle() {
    return rotateToAngle(() -> ShooterPivotS.Constants.AMP_ANGLE);
  }

  public Command rotateWithVelocity(DoubleSupplier angleSupplier, DoubleSupplier velocitySupplier) {
    return run(()->setAngle(angleSupplier.getAsDouble(), velocitySupplier.getAsDouble()));
  }
  public Command hold(){
    return sequence(
      runOnce(()->{
        resetProfile();
        setAngle(getAngle());
      }),
      run(()->setAngle(m_desiredState.position))
    );
  }

  /**
   * Calculates the voltage required to hold the pivot in its current position,
   * countering gravity.
   */
  public double getGravityFF() {
    return Constants.K_G * Math.cos(m_io.getAngle());
  }

  /**
   * Calculates the voltage required to hold the pivot in its current position,
   * countering gravity.
   */
  public double getVelocityFF() {
    return m_feedforward.calculate(m_setpoint.velocity);
  }

    public Command coast() {return 
    sequence(
      Commands.runOnce(
        ()->m_io.setIdleMode(IdleMode.kCoast)),
      Commands.idle()
    ).ignoringDisable(true).finallyDo(()->{m_io.setIdleMode(IdleMode.kBrake);});
  }

  public final Trigger atAmpAngle = new Trigger(()->Math.abs(getAngle() - Constants.AMP_ANGLE) < Units.degreesToRadians(4));
  public class Constants {
    private static final double OLD_LOWER_STOP = Units.degreesToRadians(180-19+0.75+0.4);
    private static final double NEW_LOWER_STOP = Units.degreesToRadians(161.38);
    public static final double CCW_LIMIT = NEW_LOWER_STOP;
    public static final double CW_LIMIT = Units.degreesToRadians(180-53);
    public static final double AMP_ANGLE = ShooterPivotS.Constants.CCW_LIMIT - Units.degreesToRadians(4);
    public static final int CAN_ID = 40;
    /**
     * Also equivalent to motor radians per pivot radian
     */
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 20 * 2 * 6;
    public static final double K_G = 0.06;
    public static final double K_S = 0.03;
    /**
     * Units: Volts / (Pivot radians/sec)
     * 1/((motor rad/s)/volt) = volts/(motorRad/s)
     * volts/(motorRad/s) * motorRad/pivotRad = volts/(pivotRad/s) = volts*s/pivotRad
     * 
     * Sanity check: motorRad/pivotRad > 1, so we multiply because we need more volts to get
     * a given pivot speed than to get the same motor speed.
     */
    public static final double K_V = 
      MOTOR_ROTATIONS_PER_ARM_ROTATION/(DCMotor.getNeo550(1).KvRadPerSecPerVolt); 
    public static final double K_A = 0.03;
    public static final double CG_DIST = Units.inchesToMeters(6);
    /**
     * radians per second, rad/s^2
     */
    public static final Constraints CONSTRAINTS = new Constraints(
      6, 10);
  }

}
