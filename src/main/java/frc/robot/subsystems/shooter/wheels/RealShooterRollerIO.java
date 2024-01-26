package frc.robot.subsystems.shooter.wheels;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.sparkmax.SparkDevice;

public class RealShooterRollerIO extends ShooterRollerIO {

    private CANSparkFlex m_motor;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pid;
    private double ffVolts = 0;

    public RealShooterRollerIO(int CAN_ID, boolean invert) {
        m_motor = SparkDevice.getSparkFlex(CAN_ID);
        m_motor.setInverted(invert);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_encoder = m_motor.getEncoder();
        m_pid = m_motor.getPIDController();
    }

    @Override
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public void setPIDFF(double velocity, double ffVolts) {
        this.ffVolts = ffVolts;
        m_pid.setReference(velocity, ControlType.kVelocity, 0, ffVolts);
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
    
}
