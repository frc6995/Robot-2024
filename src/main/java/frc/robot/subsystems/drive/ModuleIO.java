package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.function.Consumer;

public abstract class ModuleIO implements Logged {

  // steering trapezoid profile
  protected double m_steerSetpoint = 0;
  protected double m_driveSetpoint = 0;
  private final SimpleMotorFeedforward m_driveFeedForward =
      new SimpleMotorFeedforward(Robot.isReal() ? DRIVE_FF_CONST[0] : 0, DRIVE_FF_CONST[1], 0.2);

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

    addPeriodic.accept(this::setState);
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

  private void setState() {
    SwerveModuleState state =
        SwerveModuleState.optimize(m_desiredState, new Rotation2d(getAngle()));

    state.speedMetersPerSecond *=
        Math.cos(state.angle.minus(new Rotation2d(getAngle())).getRadians());
    double prevVelSetpoint = m_driveSetpoint;
    m_driveSetpoint = state.speedMetersPerSecond;
    double accel = (state.speedMetersPerSecond - prevVelSetpoint) / 0.02;
    setDrivePid(state.speedMetersPerSecond, m_driveFeedForward.calculate(prevVelSetpoint, accel));

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
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngle()));
  }

  @Log.NT
  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getAngle()));
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
  }

  @Log.NT
  public double getDriveCurrent() {
    return 0;
  }
}
