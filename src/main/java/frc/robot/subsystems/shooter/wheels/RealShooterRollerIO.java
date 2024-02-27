package frc.robot.subsystems.shooter.wheels;

import java.util.function.Consumer;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;

public class RealShooterRollerIO extends ShooterRollerIO {

    private CANSparkFlex m_motor;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pid;
    private double ffVolts = 0;

    class Constants {
        public static final Consumer<SparkBaseConfig> config = c->{
            c.hallEncoder.measurementPeriod(1).averageDepth(1)
            .positionConversionFactor(43.0/26.0)
            .velocityConversionFactor(43.0/26.0);
            c
                .freeLimit(60)
                .stallLimit(60)
                .status6(65535)
                .status5(65535)
                .status4(65535)
                .status3(65535)
                .status2(10);
            c.pid.p(0.0001);
            c.idleMode(IdleMode.kCoast);
        };
    }

    public RealShooterRollerIO(int CAN_ID, boolean invert) {
        m_motor = new SparkBaseConfig(Constants.config).inverted(invert)
        .applyFlex(SparkDevice.getSparkFlex(CAN_ID), true);
        m_encoder = SparkDevice.getMainEncoder(m_motor);
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
        return m_motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    }

    @Override
    public double getPidVolts() {
        return getVolts() - ffVolts;
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }
    @Override
    public double getPosition() {
        return m_encoder.getPosition();
    }
    
}
