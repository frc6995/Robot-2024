package frc.robot.subsystems.amp.pivot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.amp.pivot.AmpPivotS.Constants.*;
import static frc.robot.subsystems.amp.pivot.RealAmpPivotIO.Constants.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;

public class RealAmpPivotIO extends AmpPivotIO {
    private CANSparkMax m_motor;
    private SparkPIDController m_controller;
    private RelativeEncoder m_encoder;
    private double ffVolts;
    public RealAmpPivotIO() {
        super();
        m_motor = SparkDevice.getSparkMax(AmpPivotS.Constants.CAN_ID);
        CONFIG.apply(m_motor,true);
        m_encoder = SparkDevice.getMainEncoder(m_motor);
        m_controller = m_motor.getPIDController();
    }   

    @Override
    public double getAngle() {
        return m_encoder.getPosition();
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

    @Override
    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }
    
    public class Constants {
        /**
         * We want positive voltage to drive towards the lower hardstop.
         */
        public static final boolean INVERTED = false;
        public static final int CURRENT_LIMIT = 20;
        public static final SparkBaseConfig CONFIG = new SparkBaseConfig((c)->{
            c
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .forwardSoftLimit(Units.degreesToRadians(230))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Units.degreesToRadians(0))
            .reverseSoftLimitEnabled(true)
            .stallLimit(120)
            .freeLimit(120);
            c.pid.slot0.pidFF(0.3, 0, 0, 0);
            c.hallEncoder
            .positionConversionFactor(Units.rotationsToRadians(1.0/MOTOR_ROTATIONS_PER_ARM_ROTATION))
            .velocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1.0/MOTOR_ROTATIONS_PER_ARM_ROTATION));
        });
    }

    @Override
    public void periodic() {
    }
}
