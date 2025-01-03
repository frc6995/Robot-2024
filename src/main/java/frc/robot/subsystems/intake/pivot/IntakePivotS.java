// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.wpilibj2.command.Commands.*;


import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.ExponentialProfile;
import frc.robot.util.ExponentialProfile.Constraints;
import frc.robot.util.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.LightStripS.States;
import monologue.Logged;
import monologue.Annotations.Log;
import static edu.wpi.first.units.Units.*;

/**
 * Convention:
 * angle is measured as the robot facing right, angles increase ccw
 * intake pointing forward horizontal is 0
 * intake points toward the back of the robot so endstops are about 100-170 degrees
 * 
 */
public class IntakePivotS extends SubsystemBase implements Logged {
  private Trigger isHomed;
  private boolean hasHomed = false; 
  /**
   * IO class for interacting with motor.
   */
  private IntakePivotIO m_io;
  /**
   * The ExponentialProfile for calculating smooth movement
   */
  private ExponentialProfile m_profile;
  /**
   * The setpoint to be tracking at the moment.
   */
  private State m_setpoint = new State();

  private State m_nextSetpoint = new State();
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
  public final MechanismLigament2d INTAKE_PIVOT = new MechanismLigament2d(
    "intake", Units.inchesToMeters(6), 0, 4, new Color8Bit(235, 137, 52));
  public final MechanismLigament2d INTAKE_BEND = new MechanismLigament2d(
    "intake-bend", Units.inchesToMeters(11), -80, 4, new Color8Bit(235, 137, 52));

  /** Creates a new IntakePivotS. */
  public IntakePivotS() {
    // Create the IO class.
    if (Robot.isSimulation()) {
      m_io = new SimIntakePivotIO();
      isHomed = new Trigger(()-> true );
      hasHomed = true;
    }
    else {
      m_io = new RealIntakePivotIO();
        isHomed = new Trigger(()-> getCurrent()>10).debounce(0.6995);
    }
    m_profile = new ExponentialProfile(Constants.CONSTRAINTS);
    INTAKE_PIVOT.append(INTAKE_BEND);
    setDefaultCommand(
      sequence(
        hold().until(this::hasHomed).unless(this::hasHomed),
        retract()
      )
    ); //either(hold(), homeC().andThen(hold()), ()->hasHomed));
  }
  @Log.Once public double[] ff = new double[] {Constants.K_S, Constants.K_V, Constants.K_A, Constants.K_G};
  @Log public double getGoal() {return m_desiredState.position;}
  @Log public double getGoalVelocity() {return m_desiredState.velocity;}
  @Log public double getAngle() {return m_io.getAngle();}
  @Log public double getVelocity() {return m_io.getVelocity();}
  @Log public double getPidVolts() {return m_io.getPidVolts();}
  @Log public double getVolts() {return m_io.getVolts();}
  @Log public double getCurrent() {return m_io.getCurrent();}
  public boolean isHomed() {return isHomed.getAsBoolean();}
  public boolean hasHomed() {return hasHomed;};
  public Command coast() {return 
    sequence(
      Commands.runOnce(
        ()->m_io.setIdleMode(IdleMode.kCoast)),
      idle()
    ).ignoringDisable(true).finallyDo(()->{m_io.setIdleMode(IdleMode.kBrake);});
  }
  public void periodic() {
    // Update our visualization
    INTAKE_PIVOT.setAngle(Units.radiansToDegrees(m_io.getAngle() + Units.degreesToRadians(55-14) ));

    
    if (DriverStation.isEnabled()) {
      
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
    m_desiredState.position = angle;
    m_desiredState.velocity = 0;
      // If enabled, calculate the next step in the profile from our previous setpoint
      // to our desired state
      m_nextSetpoint = m_profile.calculate(0.03, m_setpoint, m_desiredState);
      m_setpoint = m_profile.calculate(0.02, m_setpoint, m_desiredState);
      // log that information
      
      log("setpointVelocity", m_setpoint.velocity);
      log("setpointPosition", m_setpoint.position);
      
      // Calculate the feedforward. This is partly to counter gravity
      double ffVolts = getGravityFF() + getVelocityFF();
      m_io.setPIDFF(m_setpoint.position, ffVolts);
  }
  public void resetController() {
    m_setpoint = new State(m_io.getAngle(), m_io.getVelocity());
  }

  public Command runVoltage(DoubleSupplier voltage) {
    return run(()->m_io.setVolts(voltage.getAsDouble()));
  }

  public Command rotateToAngle(DoubleSupplier angleSupplier) {
    return run(()->setAngle(angleSupplier.getAsDouble()));
  }
  public Command deploy(){
    return runOnce(this::resetController).andThen(rotateToAngle(()->Constants.CW_LIMIT));
  }
  public Command retract(){
    /* Rotated to Constants.RETRACTED instead of a specified number due to the new structure of the code */
    return runOnce(this::resetController).andThen(rotateToAngle(()->Constants.RETRACTED));
  }
  public Command hold(){
    return sequence(
      runOnce(()->setAngle(getAngle())),
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
    return m_feedforward.calculate(m_setpoint.velocity, m_nextSetpoint.velocity, 0.01);
  }

  public Command resetToRetractedC() {
    return
      deadline(
        sequence(
          Commands.runOnce(()->{m_io.resetAngle(Constants.CCW_LIMIT);}),
          waitSeconds(0.03),
          Commands.runOnce(()->{resetController();})
        ),
        runVoltage(()->0)
      )
      .ignoringDisable(true)
      .finallyDo(()-> hasHomed = true);
  }
  public Command resetToExtendedC() {
    return 
      deadline(
        sequence(
          Commands.runOnce(()->{m_io.resetAngle(Constants.CW_LIMIT);}),
          waitSeconds(0.03),
          Commands.runOnce(()->{resetController();})
        ),
        runVoltage(()->0)
      )
      .ignoringDisable(true);
  }
  public Command homeC(){
    return Commands.sequence(
      resetToExtendedC(),
      runVoltage(()->1).until(isHomed),
      resetToRetractedC()
    );
  }
  private MutableMeasure<Angle> positionMeasure = MutableMeasure.ofBaseUnits(0, Radians);
  private MutableMeasure<Velocity<Angle>> velocityMeasure = MutableMeasure.ofBaseUnits(0, RadiansPerSecond);
  private MutableMeasure<Voltage> voltsMeasure = MutableMeasure.ofBaseUnits(0, Volts);
  public SysIdRoutine m_idRoutine = new SysIdRoutine(
    new Config(
      Volts.of(0.5).per(Second),
      Volts.of(3),
      Seconds.of(10)
    ), 
    new Mechanism(
      (Measure<Voltage> volts)->m_io.setVolts(volts.in(Volts)),
      (log)->{
        log.motor("intakePivot").angularPosition(
          positionMeasure.mut_replace(getAngle(), Radians)
        ).angularVelocity(
          velocityMeasure.mut_replace(getVelocity(), RadiansPerSecond)
        ).voltage(
          voltsMeasure.mut_replace(getVolts(), Volts)
        );

      }, this, "intake"));
  public class Constants {
    /* Moved the specifed number here so it can be called by RETRACTED */
    public static final double RETRACTED = 1.958960;
    public static final double CCW_LIMIT = Units.degreesToRadians(90 + 28);
    public static final double CW_LIMIT = -0.3972;
    public static final int CAN_ID = 22;
    public static final double INTAKE_MASS_KG = Units.lbsToKilograms(7.445);
    /**
     * Also equivalent to motor radians per pivot radian
     */
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 71.429;
    // volts per (9.8 m/s^2 / CG_dist)
    public static final double K_G = 0.17 * 0.09 / Math.cos(1.333) * 71.429 / 75.0 * 1.05;
    /** from sysid 2/15/24 */
    public static final double K_S = 0.085423;
    /**
     * Units: Volts / (Pivot radians/sec)
     * 1/((motor rad/s)/volt) = volts/(motorRad/s)
     * volts/(motorRad/s) * motorRad/pivotRad = volts/(pivotRad/s) = volts*s/pivotRad
     * 
     * Sanity check: motorRad/pivotRad > 1, so we multiply because we need more volts to get
     * a given pivot speed than to get the same motor speed.
     */
    /** from sysid 2/15/24 */
    public static final double K_V = 1.3684;
      //MOTOR_ROTATIONS_PER_ARM_ROTATION/(DCMotor.getNEO(1).KvRadPerSecPerVolt); 
      public static final double CG_DIST = Units.inchesToMeters(10);
    /** from sysid 2/15/24 */
    public static final double K_A = 0.12185 * 0.1;
    
    /**
     * radians per second, rad/s^2
     */
    public static final Constraints CONSTRAINTS = Constraints.fromCharacteristics(6-K_G, K_V, K_A);
  }

}
