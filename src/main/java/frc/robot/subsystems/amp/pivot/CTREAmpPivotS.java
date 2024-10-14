package frc.robot.subsystems.amp.pivot;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.ExponentialProfile.State;
import monologue.Annotations.Log;
import monologue.Logged;

public class CTREAmpPivotS extends SubsystemBase implements Logged {
  private final SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(
      Constants.PLANT,
      DCMotor.getKrakenX60(1),
      Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION,
      Constants.CG_DIST,
      Constants.CW_LIMIT,
      Constants.CCW_LIMIT,
      false,
      Constants.CW_LIMIT,
      null);

  /**
   * For visualization.
   */
  public final MechanismLigament2d AMP_PIVOT = new MechanismLigament2d(
      "amp", Units.inchesToMeters(6), 0, 4, new Color8Bit(235, 137, 52));
  private TalonFX m_motor = new TalonFX(Constants.CAN_ID);
  private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);
  private VoltageOut m_voltageReq = new VoltageOut(0);
  private StatusSignal<Double> m_angleSig = m_motor.getPosition();
  private double m_setpointRotations;
  public final Trigger onTarget = new Trigger(() -> Math.abs(error()) < Units.degreesToRotations(4));

  public CTREAmpPivotS() {
    var config = new TalonFXConfiguration();
    m_motor.getConfigurator().refresh(config);

    m_motor.getConfigurator().apply(Constants.configureMotor(config));
    m_motor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    setDefaultCommand(hold());
  }

  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_angleSig);
    // Update our visualization
    AMP_PIVOT.setAngle(Units.rotationsToDegrees(m_angleSig.getValueAsDouble()));
  }

  @Log
  public boolean onTarget() {
    return onTarget.getAsBoolean();
  }

  @Log
  public double setpoint() {
    return m_setpointRotations;
  }

  @Log
  public double error() {
    return m_setpointRotations - m_angleSig.getValueAsDouble();
  }

  public void simulationPeriodic() {
    for (int i = 0; i < 2; i++) {
      var simState = m_motor.getSimState();
      simState.setSupplyVoltage(12);
      // simState.getMotorVoltage is counterclockwise negative
      double volts = simState.getMotorVoltage();
      log("simVolts", volts);
      log("effectiveSimVolts", NomadMathUtil.subtractkS(volts, Constants.K_S));
      m_pivotSim.setInput(NomadMathUtil.subtractkS(volts, Constants.K_S) - Constants.K_G * Math.cos(getAngleRadians()));
      m_pivotSim.update(0.01);
      var rotorPos = m_pivotSim.getAngleRads() * Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION / (2 * Math.PI);
      var rotorVel = m_pivotSim.getVelocityRadPerSec() * Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION / (2 * Math.PI);
      log("simPos", rotorPos);
      log("simVel", rotorVel);
      simState.setRawRotorPosition(rotorPos);
      simState.setRotorVelocity(rotorVel);
    }
  }

  public void setAngleRadians(double angle) {
    m_setpointRotations = Units.radiansToRotations(angle);
    m_motor.setControl(m_profileReq.withPosition(m_setpointRotations));
  }

  public void resetController() {
    m_motor.setControl(m_profileReq.withPosition(m_angleSig.getValueAsDouble()));
  }

  public Command runVoltage(DoubleSupplier voltage) {
    return run(() -> m_motor.setControl(m_voltageReq.withOutput(voltage.getAsDouble())));
  }

  public Command rotateToAngle(DoubleSupplier angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.getAsDouble()));
  }

  public Command deploy() {
    return runOnce(this::resetController).andThen(rotateToAngle(() -> Constants.CW_LIMIT));
  }

  public Command retract() {
    return runOnce(this::resetController).andThen(rotateToAngle(() -> 1.958960));
  }

  public Command hold() {
    return sequence(
        runOnce(() -> setAngleRadians(getAngleRadians())),
        Commands.idle());
  }

  @Log
  public double getAngleRotations() {
    return m_angleSig.getValueAsDouble();
  }

  @Log
  public double getAngleRadians() {
    return Units.rotationsToRadians(m_angleSig.getValueAsDouble());
  }

  public Command resetToRetractedC() {
    return deadline(
        sequence(
            Commands.runOnce(() -> {
              m_motor.setPosition((Constants.CW_LIMIT));
            }),
            waitSeconds(0.03),
            Commands.runOnce(() -> {
              resetController();
            })),
        runVoltage(() -> 0))
        .ignoringDisable(true);
  }

  public class Constants {
    public static final double CCW_LIMIT = Units.degreesToRadians(230);
    public static final double CW_LIMIT = Units.degreesToRadians(0);
    public static final double SCORE_ANGLE = 2 * Math.PI / 3.0;
    public static final int CAN_ID = 42;
    /**
     * Also equivalent to motor radians per pivot radian
     */
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 44.0 / 16.0 * 30.0 / 18.0;
    // ks + kg = 0.83
    public static final double K_G = 0.60;
    public static final double K_S = 0.2;
    /**
     * Units: Volts / (Pivot Rotations/sec)
     */
    public static final double K_V = 0.55;
    public static final double K_A = 0.13;
    public static final double CG_DIST = Units.inchesToMeters(6);
    public static final LinearSystem<N2, N1, N1> PLANT = LinearSystemId
        .identifyPositionSystem(Units.radiansToRotations(K_V), Units.radiansToRotations(K_A));

    public static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
      config.MotorOutput
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake);
      config.Feedback.withSensorToMechanismRatio(Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION);
      config.SoftwareLimitSwitch
          .withForwardSoftLimitThreshold(Units.radiansToRotations(Constants.CCW_LIMIT))
          .withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(Units.radiansToRotations(Constants.CW_LIMIT))
          .withReverseSoftLimitEnable(true);
      config.CurrentLimits
          .withStatorCurrentLimit(100)
          .withStatorCurrentLimitEnable(true);
      config.Slot0
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withKS(0.59).withKV(Constants.K_V).withKA(Constants.K_A)
          .withKP(Robot.isReal() ? 30 : 3).withKI(0).withKD(0.001).withKG(Constants.K_G);
      config.MotionMagic
          .withMotionMagicCruiseVelocity(3)
          .withMotionMagicAcceleration(6);
      return config;
    }
  }
}
