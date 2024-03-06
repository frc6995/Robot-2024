package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.function.Consumer;

public abstract class ModuleIO implements Logged {

  // steering trapezoid profile
  protected double m_steerSetpoint = 0;
  protected double m_driveSetpoint = 0;
  protected SwerveModulePosition m_currentPosition = new SwerveModulePosition();
  protected SwerveModuleState m_currentState = new SwerveModuleState();
  @Log
  protected double ffVolts = 0;
  @Log
  protected double pidVolts = 0;
  private final SimpleMotorFeedforward m_driveFeedForward =
      new SimpleMotorFeedforward(Robot.isReal() ? DRIVE_FF_CONST[0] : 0, DRIVE_FF_CONST[1], DRIVE_FF_CONST[2]);

  private final SimpleMotorFeedforward m_steerFeedForward =
      new SimpleMotorFeedforward(0.000, STEER_KV, 0.001);

  private String m_loggingName;
  // steering PID controller
  // drive PID controller
  protected final ModuleConstants m_moduleConstants;

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  public ModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
    m_moduleConstants = moduleConstants;
    m_loggingName =
        moduleConstants.name
            + "-["
            + moduleConstants.driveMotorID
            + ','
            + moduleConstants.rotationMotorID
            + ']';
  }

  public String getPath() {
    System.out.println(m_loggingName);
    return m_loggingName;
  }

  public abstract void setDriveVoltage(double driveVolts);

  public abstract void setRotationVoltage(double rotationVolts);

  @Log.NT
  public abstract double getDriveDistance();

  @Log.NT
  public abstract double getDriveVelocity();

  
  /**
   * Returns the angle of the module from pi to -pi
   *
   * @return
   */
  @Log.NT
  public abstract double getAngle();

  @Log.NT
  public abstract double getRelativeAngle();

  @Log.NT
  public abstract double getDriveVoltage();

  @Log.NT
  public abstract double getSteerVoltage();

  @Log.NT
  public double getSteerSetpoint() {
    return m_steerSetpoint;
  }

  @Log.NT
  public double getSteerCurrent() {
    return 0;
  }

  @Log.NT
  public double getDriveSetpoint() {
    return m_driveSetpoint;
  }

  public void setDesiredState(SwerveModuleState state) {
    m_desiredState = state;
  }

  public void updateInputs() {};
  public void setState() {
    SwerveModuleState state =
    SwerveModuleState.optimize(m_desiredState, new Rotation2d(getAngle()));

    state.speedMetersPerSecond *= Math.cos(state.angle.minus(new Rotation2d(getAngle())).getRadians());
    double prevVelSetpoint = m_driveSetpoint;
    log("prevSetpt", prevVelSetpoint);
    m_driveSetpoint = state.speedMetersPerSecond;
    log("setpt", m_driveSetpoint);
    double accel = (m_driveSetpoint - prevVelSetpoint) / 0.02;
    log("accel", accel);
    if (DriverStation.isTeleop()) {
      //accel = 0;
    }
    setDrivePid(state.speedMetersPerSecond, m_driveFeedForward.calculate(m_driveSetpoint, accel));

    m_steerSetpoint = state.angle.getRadians();
    // Get error which is the smallest distance between goal and measurement
    double errorBound = Math.PI;
    double goalMinDistance =
        MathUtil.inputModulus(m_steerSetpoint - getAngle(), -errorBound, errorBound);

    m_steerSetpoint = goalMinDistance + getAngle();

    setRotationPid(m_steerSetpoint, 0);
  }

  public abstract void setDrivePid(double velocity, double ffVolts);

  public abstract void setRotationPid(double angle, double ffVolts);

  @Log.NT
  public SwerveModuleState getCurrentState() {
    m_currentState.speedMetersPerSecond = getDriveVelocity();
    m_currentState.angle = new Rotation2d(getRelativeAngle());
    return m_currentState;
  }

  @Log.NT
  public SwerveModulePosition getCurrentPosition() {
    m_currentPosition.distanceMeters = getDriveDistance();
    m_currentPosition.angle = new Rotation2d(getRelativeAngle());
    return m_currentPosition;
  }

  public double getPinionSlip() {
    return MathUtil.angleModulus(getRelativeAngle() - getAngle());
  }

  /** Reset driven distance to 0 meters. */
  public abstract void resetDistance();

  /** Reset */
  public abstract void reinitRotationEncoder();

  public void resetSteerController() {
    m_steerSetpoint = getAngle();
    m_desiredState.angle = new Rotation2d(getAngle());
  }

  @Log.NT
  public double getDriveCurrent() {
    return 0;
  }

  
  private MutableMeasure<Voltage> voltMeasure = MutableMeasure.ofBaseUnits(0, Volts);
  private MutableMeasure<Distance> distanceMeasure = MutableMeasure.ofBaseUnits(0, Meters);
  private MutableMeasure<Velocity<Distance>> velocityMeasure = MutableMeasure.ofBaseUnits(0, MetersPerSecond);
  private String sysIdDriveName;
  public void logDriveMotor(SysIdRoutineLog log) {
    log.motor(sysIdDriveName)
      .linearPosition(distanceMeasure.mut_replace(getDriveDistance(), Meters))
      .voltage(voltMeasure.mut_replace(getDriveVoltage(), Volts))
      .linearVelocity(velocityMeasure.mut_replace(getDriveVelocity(), MetersPerSecond));
  }
}
