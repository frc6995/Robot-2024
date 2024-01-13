package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.util.sim.SparkMaxAbsoluteEncoderWrapper;
import frc.robot.util.sparkmax.SparkDevice;

import java.util.function.Consumer;

public class RealModuleIO extends ModuleIO {

  protected final CANSparkMax m_driveMotor;
  protected final CANSparkMax m_steerMotor;
  protected final SparkPIDController m_driveController;
  protected final SparkPIDController m_rotationController;
  protected final SparkMaxAbsoluteEncoderWrapper m_magEncoder;
  protected double m_driveDistance = 0;
  protected double m_driveVelocity = 0;
  protected double m_steerAngle = 0;
  protected double m_steerVolts = 0;
  protected double m_driveVolts = 0;
  protected RelativeEncoder m_driveEncoder;

  public RealModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
    super(addPeriodic, moduleConstants);
    m_driveMotor = SparkDevice.getSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    m_steerMotor = SparkDevice.getSparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
    var magEncoder = m_steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // Drive motor config
    m_driveMotor.setSmartCurrentLimit(50);
    m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    var error = m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    System.out.println("drive config " + m_moduleConstants.driveMotorID + error.toString());
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(
        Math.PI
            * (WHEEL_RADIUS_M * 2) // meters/ wheel rev
            / WHEEL_ENC_COUNTS_PER_WHEEL_REV // 1/ (enc revs / wheel rev) = wheel rev/enc rev
        );
    m_driveEncoder.setVelocityConversionFactor(
        (WHEEL_RADIUS_M * 2) * Math.PI / 60 / WHEEL_ENC_COUNTS_PER_WHEEL_REV);

    m_driveMotor.setIdleMode(IdleMode.kBrake);
    Timer.delay(0.1);
    // Steer motor config
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    m_steerMotor.setSmartCurrentLimit(40);
    m_steerMotor.getEncoder().setPositionConversionFactor(2.0 * Math.PI * AZMTH_REVS_PER_ENC_REV);
    magEncoder.setPositionConversionFactor(Math.PI * 2);
    magEncoder.setVelocityConversionFactor(Math.PI * 2 * 60);
    m_magEncoder =
        new SparkMaxAbsoluteEncoderWrapper(m_steerMotor, m_moduleConstants.magEncoderOffset);
    m_steerMotor.setIdleMode(IdleMode.kBrake);
    m_steerMotor.setInverted(true);
    m_driveController = m_driveMotor.getPIDController();
    m_rotationController = m_steerMotor.getPIDController();
    m_driveController.setFeedbackDevice(m_driveMotor.getEncoder());
    m_rotationController.setFeedbackDevice(m_steerMotor.getAbsoluteEncoder(Type.kDutyCycle));
    m_rotationController.setPositionPIDWrappingEnabled(true);
    m_rotationController.setPositionPIDWrappingMaxInput(Math.PI);
    m_rotationController.setPositionPIDWrappingMinInput(-Math.PI);

    m_driveController.setP(0.1);
    m_driveController.setI(0);
    m_driveController.setD(0.005);

    m_rotationController.setP(0.5);
    m_rotationController.setI(0);
    m_rotationController.setD(0);
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
