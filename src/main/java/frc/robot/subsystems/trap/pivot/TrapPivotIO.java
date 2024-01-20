package frc.robot.subsystems.trap.pivot;

import monologue.Logged;

public abstract class TrapPivotIO implements Logged {
    public TrapPivotIO(){}
    public abstract double getAngle();
    public abstract void setPIDFF(double angle, double ffVolts);
    public abstract void resetAngle(double angle);
    public abstract void periodic();
    public abstract void setVolts(double volts);
    public abstract double getVolts();
    public abstract double getPidVolts();
}
