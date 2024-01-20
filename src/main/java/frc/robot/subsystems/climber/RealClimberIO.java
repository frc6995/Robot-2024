package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.climber.ClimberS.Constants.*;
import static frc.robot.subsystems.climber.RealClimberIO.Constants.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.sparkmax.SparkDevice;

public class RealClimberIO extends ClimberIO {
    private CANSparkMax m_leader;
    private CANSparkMax m_follower;
    private SparkPIDController m_controller;
    private RelativeEncoder m_encoder;
    private double ffVolts;
    public RealClimberIO() {
        super();
        m_leader = SparkDevice.getSparkMax(ClimberS.Constants.LEADER_CAN_ID);
        m_follower = SparkDevice.getSparkMax(ClimberS.Constants.FOLLOWER_CAN_ID);
        m_leader.setIdleMode(IdleMode.kBrake);
        m_follower.setIdleMode(IdleMode.kBrake);
        m_leader.setInverted(INVERTED);
        m_leader.setSmartCurrentLimit(CURRENT_LIMIT);
        m_follower.setSmartCurrentLimit(CURRENT_LIMIT);
        m_follower.follow(m_leader);
        m_controller = m_leader.getPIDController();
        m_encoder = m_leader.getEncoder();
        m_encoder.setPositionConversionFactor(1.0/MOTOR_ROTATIONS_PER_METER);
        m_encoder.setVelocityConversionFactor(1.0/MOTOR_ROTATIONS_PER_METER/60.0);
        m_controller.setP(kP);
        m_controller.setI(kI);
        m_controller.setD(kD);
        m_controller.setFF(0);
    }

    @Override
    public double getLength() {
        return m_encoder.getPosition();
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
        public static final int CURRENT_LIMIT = 20;
    }
}
