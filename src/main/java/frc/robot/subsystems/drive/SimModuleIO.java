package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.util.sim.wpiClasses.QuadSwerveSim;
import frc.robot.util.sim.wpiClasses.SwerveModuleSim;
import java.util.function.Consumer;

public class SimModuleIO extends ModuleIO {

  SwerveModuleSim moduleSim = SimModuleIO.swerveSimModuleFactory();
  private double m_steerVolts;
  private double m_driveVolts;

  private final PIDController m_steerPIDController;
  // logging position error because it's actually the "process variable", vs its
  // derivative
  private final PIDController m_drivePIDController;

  public SimModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
    super(addPeriodic, moduleConstants);
    moduleSim.resetAzmth(Math.random() * 2 * Math.PI);
    m_steerPIDController = new PIDController(10, 0.0, STEER_D);
    // Tell the PID controller that it can move across the -pi to pi rollover point.
    m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // For a velocity controller we just use P
    // (and feedforward, which is handled in #setDesiredStateClosedLoop)
    m_drivePIDController = new PIDController(DRIVE_P, 0, DRIVE_D);
  }

  static SwerveModuleSim swerveSimModuleFactory() {
    return new SwerveModuleSim(
        DCMotor.getNeoVortex(1),
        DCMotor.getNeoVortex(1),
        WHEEL_RADIUS_M,
        1.0 / AZMTH_REVS_PER_ENC_REV, // steering motor rotations per wheel steer rotation
        1.0 / WHEEL_REVS_PER_ENC_REV,
        1.0 / AZMTH_REVS_PER_ENC_REV, // same as motor rotations because NEO encoder is on motor
        // shaft
        1.0 / WHEEL_REVS_PER_ENC_REV,
        0.9,
        0.8,
        ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES,
        0.01);
  }

  public SwerveModuleSim getModuleSim() {
    return moduleSim;
  }

  public void setDrivePid(double velocity, double ffVolts) {
    this.ffVolts = ffVolts;
    pidVolts = m_drivePIDController.calculate(getDriveVelocity(), velocity);
    setDriveVoltage(pidVolts + ffVolts);
  }

  public void setRotationPid(double angle, double ffVolts) {
    double pidVolts = m_steerPIDController.calculate(getAngle(), angle);
    setRotationVoltage(pidVolts + ffVolts);
  }

  public void setRotationVoltage(double volts) {
    if (DriverStation.isDisabled()) {
      volts = 0;
    }
    m_steerVolts = volts;
    moduleSim.setAzmthVoltage(m_steerVolts);
  }

  public void setDriveVoltage(double volts) {
    if (DriverStation.isDisabled()) {
      volts = 0;
    }
    m_driveVolts = MathUtil.clamp(volts, -12, 12);
    moduleSim.setWheelVoltage(m_driveVolts);
  }

  @Override
  public double getAngle() {
    return MathUtil.angleModulus(
        moduleSim.getAzimuthEncoderPositionRev() / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI);
  }

  @Override
  public double getDriveDistance() {
    return moduleSim.getWheelEncoderPositionRev()
        / WHEEL_ENC_COUNTS_PER_WHEEL_REV
        * 2
        * Math.PI
        * WHEEL_RADIUS_M;
  }

  @Override
  public double getDriveVelocity() {
    return moduleSim.getWheelEncoderVelocityRevPerSec()
        / WHEEL_ENC_COUNTS_PER_WHEEL_REV
        * 2
        * Math.PI
        * WHEEL_RADIUS_M;
  }

  @Override
  public void resetDistance() {
    moduleSim.resetWheel(0);
  }

  @Override
  public void reinitRotationEncoder() {}

  @Override
  public double getDriveVoltage() {
    return m_driveVolts;
  }

  @Override
  public double getSteerVoltage() {
    return m_steerVolts;
  }

  @Override
  public double getRelativeAngle() {

    return moduleSim.getAzimuthEncoderPositionRev() / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI;
  }
}
