package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import monologue.Logged;
import monologue.Annotations.Log;

import frc.robot.NavX.AHRS;
import java.util.List;
import java.util.function.Consumer;

public abstract class SwerveDriveIO implements Logged {
  protected final AHRS m_navx = new AHRS(Port.kMXP, (byte) 50);
  protected List<ModuleIO> m_modules;
  public boolean isSysid = false;

  @Log.NT
  private SwerveModuleState[] currentStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  @Log.NT
  private SwerveModulePosition[] currentPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  public SwerveDriveIO(Consumer<Runnable> addPeriodic) {
    addPeriodic.accept(this::periodic);
    m_navx.reset();
    m_navx.enableLogging(true);
  }

  @Override
  public String getPath() {
    return "io";
  }

  @Log.NT
  public Rotation2d getGyroHeading() {
    return new Rotation2d(Units.degreesToRadians(-m_navx.getAngle()));
  }

  private void periodic() {
    updateModulePositions();
    updateModuleStates();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates, boolean isSysid) {
    this.isSysid = isSysid;
    for (int i = 0; i < NUM_MODULES; i++) {
      m_modules.get(i).setDesiredState(moduleStates[i]);
    }
  }

  public void sysidLinear(double volts) {
    for (ModuleIO mod : m_modules) {
      mod.setRotationPid(0, 0);
      mod.setDriveVoltage(volts);
    }
  }

  public void sysidAngular(double volts) {
    // FL
    m_modules.get(0).setRotationPid(3* Math.PI / 4.0, 0);
    // FR
    m_modules.get(1).setRotationPid(Math.PI / 4.0, 0);
    // BL
    m_modules.get(2).setRotationPid(5*Math.PI / 4.0, 0);
    m_modules.get(3).setRotationPid(7*Math.PI/4.0, 0);
    for (ModuleIO mod : m_modules) {
      mod.setDriveVoltage(volts);
    }
  }

  @Log.NT
  public SwerveModuleState[] getModuleStates() {
    return currentStates;
  }
  @Log.NT
  public SwerveModulePosition[] getCurrentPositions() {
    return currentPositions;
  }

  private void updateModuleStates() {
    for (int i = 0; i < NUM_MODULES; i++) {
      currentStates[i] = m_modules.get(i).getCurrentState();
    }
  }

  private void updateModulePositions() {
    for (int i = 0; i < NUM_MODULES; i++) {
      currentPositions[i] = m_modules.get(i).getCurrentPosition();
    }
  }

  public void resetDistances() {
    m_modules.forEach((module) -> module.resetDistance());
  }

  public abstract void resetPose(Pose2d pose);

  public abstract void resetIMU();

  public void reinitRotationEncoders() {
    m_modules.forEach(ModuleIO::reinitRotationEncoder);
  }

  /** Reset */
  public void resetModuleSteerControllers() {
    m_modules.forEach(ModuleIO::resetSteerController);
  }
  @Log.NT
  public abstract Pose2d getSimPose();

  public Rotation3d getRotation3d() {
    return new Rotation3d(
        new Quaternion(
            m_navx.getQuaternionW(),
            m_navx.getQuaternionX(),
            m_navx.getQuaternionY(),
            m_navx.getQuaternionZ()));
  }

  public double getPitch() {
    return Units.degreesToRadians(m_navx.getPitch());
  }

  public void logDriveMotors(SysIdRoutineLog log) {
    m_modules.forEach(m->m.logDriveMotor(log));
  }
}
