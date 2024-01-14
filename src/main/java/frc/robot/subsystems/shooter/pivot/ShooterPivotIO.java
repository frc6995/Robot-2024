package frc.robot.subsystems.shooter.pivot;

import monologue.Logged;
import monologue.Annotations.Log;

public abstract class ShooterPivotIO implements Logged {

    public ShooterPivotIO(){}
    @Log.NT
    public abstract double getAngle();
    public abstract void setPIDFF(double angle, double ffVolts);
    public abstract void resetAngle(double angle);
    public abstract void periodic();
    public abstract void setVolts(double volts);
    @Log.NT
    public abstract double getVolts();
}
