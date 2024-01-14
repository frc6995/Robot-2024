package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.util.sparkmax.SparkDevice;

public class RealShooterPivotIO extends ShooterPivotIO {
    private CANSparkMax m_motor;
    private SparkPIDController m_controller;
    private SparkRelativeEncoder m_encoder;
    public RealShooterPivotIO() {
        super();
        m_motor = SparkDevice.getSparkMax(ShooterPivotS.Constants.CAN_ID);
        m_controller = m_motor.getPIDController();
    }

    @Override
    public double getAngle() {
        return m_encoder.getPosition();
    }

    @Override
    public void setPIDFF(double angle, double ffVolts) {
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
    
}
