package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.climber.ClimberS.Constants.*;
import static frc.robot.subsystems.climber.RealClimberIO.Constants.*;

import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;

public class RealClimberIO extends ClimberIO {
    private CANSparkFlex m_leader;
    private SparkPIDController m_controller;
    private RelativeEncoder m_encoder;
    private double ffVolts;
    public RealClimberIO(boolean isLeft) {
        super(isLeft);
    m_leader =     new SparkBaseConfig (Constants.config)
    .inverted(!isLeft)
    .applyFlex(
    SparkDevice.getSparkFlex(isLeft ? LEFT_CAN_ID : RIGHT_CAN_ID, MotorType.kBrushless),
    true
    );
        m_controller = m_leader.getPIDController();
        m_encoder = SparkDevice.getMainEncoder(m_leader);
        m_encoder.setPosition(LOWER_LIMIT);
    }

    @Override
    public double getLength() {
        return m_encoder.getPosition();
    }
    public short getFaults() {
        return m_leader.getFaults();
    }

    @Override
    public void setPIDFF(double length, double ffVolts) {
        this.ffVolts = ffVolts;
        m_controller.setReference(length, ControlType.kPosition, 0, ffVolts);
    }

    public void resetLength(double length) {
        m_encoder.setPosition(length);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        m_leader.setIdleMode(mode);
    }

    @Override
    public void setVolts(double volts) {
        m_leader.setVoltage(volts);
    }

    @Override
    public double getVolts() {
        return m_leader.getAppliedOutput() * 12;
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
         * We want positive voltage to drive up.
         */
        public static final boolean INVERTED = false;
        public static final int CURRENT_LIMIT = 5;

        public static final Consumer<SparkBaseConfig> config = c->{
            c
                .idleMode(IdleMode.kBrake)
                .freeLimit(CURRENT_LIMIT)
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(UPPER_LIMIT)
                .reverseSoftLimit(LOWER_LIMIT)
                .reverseSoftLimitEnabled(true)
                .status6(32767)
                .status5(32767)
                .status4(32767)
                .status3(32767);
            c.pid.pidFF(0.1, 0, 0, 0);
            c.hallEncoder.positionConversionFactor(1.0/MOTOR_ROTATIONS_PER_METER);
        };
    }
}
