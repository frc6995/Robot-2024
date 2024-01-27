package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.util.sim.SparkMaxAbsoluteEncoderWrapper;
import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;
import lib.sparkmax.PIDSlotConfig;
import lib.sparkmax.PIDControllerConfig.FeedbackDevice;

import java.util.function.Consumer;

public class RealModuleIO extends ModuleIO {

  public static class Constants {
    public static SparkBaseConfig DRIVE_CONFIG = new SparkBaseConfig((c)->{
      c.
        freeLimit(50).
        idleMode(IdleMode.kBrake);//.
        //statusFrames(40, 20, 20, 65535, 65535, 65535, 65535);
      c.hallEncoder.
        positionConversionFactor(
          Math.PI * (WHEEL_RADIUS_M * 2) / WHEEL_ENC_COUNTS_PER_WHEEL_REV
        ).
        velocityConversionFactor((WHEEL_RADIUS_M * 2) * Math.PI / 60 / WHEEL_ENC_COUNTS_PER_WHEEL_REV);
      c.pid.
        pidFF(
          0.1,
          0,
          0.005,
          0).
        feedbackSensor(FeedbackDevice.kHallSensor);
    });

    public static SparkBaseConfig STEER_CONFIG = new SparkBaseConfig((c)->{
      c.
        freeLimit(40).
        inverted(true).
        idleMode(IdleMode.kBrake);
        //.statusFrames(40, 65535, 65535, 65535, 65535, 40, 65535);
      c.absEncoder.
        positionConversionFactor(Math.PI * 2).
        velocityConversionFactor(Math.PI * 2 * 60).
        inverted(true);
      c.hallEncoder.
        positionConversionFactor(Math.PI * 2 * AZMTH_REVS_PER_ENC_REV);
      c.pid.
        pidFF(0.5, 0, 0, 0).
        feedbackSensor(FeedbackDevice.kAbsoluteEncoder).
        wrappingEnabled(true).
        wrappingMaxInput(Math.PI).
        wrappingMinInput(-Math.PI);
    });
  }

  protected final CANSparkFlex m_driveMotor;
  protected final CANSparkMax m_steerMotor;
  protected final SparkPIDController m_driveController;
  protected final SparkPIDController m_rotationController;
  protected final SparkAbsoluteEncoder m_magEncoder;
  protected double m_driveDistance = 0;
  protected double m_driveVelocity = 0;
  protected double m_steerAngle = 0;
  protected double m_steerVolts = 0;
  protected double m_driveVolts = 0;
  protected RelativeEncoder m_driveEncoder;

  public RealModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
    super(addPeriodic, moduleConstants);
    m_driveMotor = Constants.DRIVE_CONFIG.applyFlex(
      SparkDevice.getSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless),
      false
    );
    m_steerMotor = Constants.STEER_CONFIG
      .copy(c->c.absEncoder.zeroOffset(m_moduleConstants.magEncoderOffset))
      .applyMax(
      SparkDevice.getSparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless),
      false
    );
    m_driveEncoder = m_driveMotor.getEncoder();
    m_magEncoder = m_steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_driveController = m_driveMotor.getPIDController();
    m_rotationController = m_steerMotor.getPIDController();
    resetDistance();
    reinitRotationEncoder();
    addPeriodic.accept(this::updateEncoders);
  }

  public void updateEncoders() {
    m_driveVolts = m_driveMotor.getAppliedOutput() * 12;
    m_steerVolts = m_steerMotor.getAppliedOutput() * 12;
    m_driveDistance = m_driveEncoder.getPosition();
    m_driveVelocity = m_driveEncoder.getVelocity();
    m_steerAngle = MathUtil.angleModulus(m_magEncoder.getPosition());
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    m_driveMotor.setVoltage(driveVolts);
  }

  @Override
  public void setRotationVoltage(double rotationVolts) {
    m_steerMotor.setVoltage(rotationVolts);
  }

  @Override
  public double getDriveDistance() {
    return m_driveDistance;
  }

  @Override
  public double getDriveVelocity() {

    return m_driveVelocity;
  }

  @Override
  public double getAngle() {

    return m_steerAngle;
  }

  @Override
  public double getRelativeAngle() {

    return 0; // m_steerMotor.getEncoder().getPosition();
  }

  @Override
  public void resetDistance() {
    m_driveMotor.getEncoder().setPosition(0);
  }

  @Override
  public void reinitRotationEncoder() {
    m_steerMotor.getEncoder().setPosition(getAngle());
  }

  @Override
  public double getDriveVoltage() {
    return m_driveVolts;
  }

  @Override
  public double getSteerVoltage() {
    return m_steerVolts;
  }

  @Override
  public double getDriveCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  @Override
  public double getSteerCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  @Override
  public void setRotationPid(double angle, double ffVolts) {
    m_rotationController.setReference(angle, ControlType.kPosition, 0, ffVolts);
  }

  @Override
  public void setDrivePid(double velocity, double ffVolts) {
    m_driveController.setReference(velocity, ControlType.kVelocity, 0, ffVolts);
  }
}
