package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;

import monologue.Logged;

public abstract class ClimberIO implements Logged {
    public ClimberIO(boolean isLeft){}
    public abstract double getLength();
    public abstract void setPIDFF(double length, double ffVolts);
    public abstract void resetLength(double length);
    public abstract void periodic();
    public abstract void setVolts(double volts);
    public abstract double getVolts();
    public abstract double getPidVolts();
    public abstract short getFaults();
    public abstract void setIdleMode(IdleMode mode);
}
