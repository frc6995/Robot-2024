package frc.robot.subsystems.trap.pivot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.trap.pivot.TrapPivotS.Constants.*;
import static frc.robot.subsystems.trap.pivot.RealTrapPivotIO.Constants.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.sparkmax.SparkDevice;

public class RealTrapPivotIO extends TrapPivotIO {
    private CANSparkFlex m_motor;
    private SparkPIDController m_controller;
    private RelativeEncoder m_encoder;
    private double ffVolts;
    public RealTrapPivotIO() {
        super();
        m_motor = SparkDevice.getSparkFlex(TrapPivotS.Constants.CAN_ID);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(INVERTED);
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
        m_controller = m_motor.getPIDController();
        m_encoder = SparkDevice.getMainEncoder(m_motor);
        m_encoder.setPositionConversionFactor(Units.rotationsToRadians(1.0/MOTOR_ROTATIONS_PER_ARM_ROTATION));
        m_encoder.setVelocityConversionFactor(
            Units.rotationsPerMinuteToRadiansPerSecond(1.0/MOTOR_ROTATIONS_PER_ARM_ROTATION)
        );
        m_controller.setP(kP);
        m_controller.setI(kI);
        m_controller.setD(kD);
        m_controller.setFF(0);
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
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        /**
         * We want positive voltage to drive towards the lower hardstop.
         */
        public static final boolean INVERTED = false;
        public static final int CURRENT_LIMIT = 20;
    }
}
