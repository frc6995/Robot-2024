package frc.robot.subsystems.intake.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.intake.pivot.IntakePivotS.Constants.*;
import static frc.robot.subsystems.intake.pivot.RealIntakePivotIO.Constants.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import frc.robot.util.sparkmax.SparkDevice;

public class RealIntakePivotIO extends IntakePivotIO {
    private CANSparkMax m_motor;
    private SparkPIDController m_controller;
    private RelativeEncoder m_encoder;
    private double ffVolts;
    private DigitalInput m_coastModeInput = new DigitalInput(0);
    private Trigger m_coastModeButton = new Trigger(m_coastModeInput::get).negate();
    public RealIntakePivotIO() {
        super();
        m_motor = SparkDevice.getSparkMax(IntakePivotS.Constants.CAN_ID);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(INVERTED);
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);

        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_controller = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(Units.rotationsToRadians(1.0/MOTOR_ROTATIONS_PER_ARM_ROTATION));
        m_encoder.setVelocityConversionFactor(
            Units.rotationsPerMinuteToRadiansPerSecond(1.0/MOTOR_ROTATIONS_PER_ARM_ROTATION)
        );
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) CCW_LIMIT);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) CW_LIMIT);
        m_controller.setP(kP);
        m_controller.setI(kI);
        m_controller.setD(kD);
        m_controller.setFF(0);
        m_coastModeButton
            .onTrue(
            Commands.runOnce(()->{m_motor.setIdleMode(IdleMode.kCoast);}).ignoringDisable(true))
            .onFalse(Commands.runOnce(()->{m_motor.setIdleMode(IdleMode.kBrake);}).ignoringDisable(true));
    }

    @Override
    public double getAngle() {
        return m_encoder.getPosition();
    }
    @Override
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public void setPIDFF(double angle, double ffVolts) {
        this.ffVolts = ffVolts;
        m_controller.setReference(angle, ControlType.kPosition, 0, ffVolts);
    }

    public void resetAngle(double angle) {
        m_encoder.setPosition(angle);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            this.ffVolts = 0;    
        }
    }

    @Override
    public void setVolts(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public double getVolts() {
        return m_motor.getAppliedOutput() * 12;
    }

    @Override
    public double getPidVolts() {
        return getVolts() - ffVolts;
    }
    
    public class Constants {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        /**
         * We want positive voltage to drive towards the retracted end.
         */
        public static final boolean INVERTED = true;
        public static final int CURRENT_LIMIT = 20;
    }
}
