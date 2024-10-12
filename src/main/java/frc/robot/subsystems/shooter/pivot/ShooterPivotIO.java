package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase.IdleMode;

import monologue.Logged;

public abstract class ShooterPivotIO implements Logged {
    public ShooterPivotIO(){}
    public abstract double getAngle();
    public abstract double getVelocity();
    public abstract void setPIDFF(double angle, double ffVolts);
    public abstract void resetAngle(double angle);
    public abstract void periodic();
    public abstract void setVolts(double volts);
    public abstract double getVolts();
    public abstract double getPidVolts();
    public abstract double getCurrent();
    public void setIdleMode(IdleMode mode){}
}
