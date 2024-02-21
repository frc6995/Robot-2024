package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.shooter.pivot.ShooterPivotS.Constants.*;

import java.util.function.Consumer;

import static frc.robot.subsystems.shooter.pivot.RealShooterPivotIO.Constants.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;
import lib.sparkmax.PIDControllerConfig.FeedbackDevice;

public class RealShooterPivotIO extends ShooterPivotIO {
    private CANSparkMax m_motor;
    private SparkPIDController m_controller;
    private RelativeEncoder m_encoder;
    private double ffVolts;
    public RealShooterPivotIO() {
        super();
        m_motor = new SparkBaseConfig(Constants.config).applyMax(
            SparkDevice.getSparkMax(ShooterPivotS.Constants.CAN_ID), true
        );
        m_controller = m_motor.getPIDController();
        m_encoder = m_motor.getAlternateEncoder(8192);
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
    public void periodic() {
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
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
        /**
         * We want positive voltage to drive towards the lower hardstop.
         */
        public static final boolean INVERTED = false;
        public static final int CURRENT_LIMIT = 20;

        public static final Consumer<SparkBaseConfig> config = c->{
            c
                .alternateEncoderMode(true)
                .inverted(false)
                .freeLimit(20)
                .stallLimit(30)
                .forwardSoftLimit(ShooterPivotS.Constants.CCW_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ShooterPivotS.Constants.CW_LIMIT)
                .reverseSoftLimitEnabled(true)
                .status6(65535)
                .status5(65535)
                .status4(10)
                .status3(65535);
            c.altEncoder
                .countsPerRev(8192)
                .positionConversionFactor(2 * Math.PI / 6.0)
                .velocityConversionFactor(2 * Math.PI / (6.0 * 60.0))
                ;
            c.pid.pidFF(kP, kI, kD, 0).feedbackSensor(FeedbackDevice.kAlternateEncoder);
            

        
        };
    }

    @Override
    public double getVelocity() {
        return m_encoder.getVelocity();
    }
}
