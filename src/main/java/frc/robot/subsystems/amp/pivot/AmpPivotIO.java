package frc.robot.subsystems.amp.pivot;

import monologue.Logged;

public abstract class AmpPivotIO implements Logged {
    public AmpPivotIO(){}
    public abstract double getAngle();
    public abstract void setPIDFF(double angle, double ffVolts);
    public abstract void resetAngle(double angle);
    public abstract void periodic();
    public abstract void setVolts(double volts);
    public abstract double getVolts();
    public abstract double getPidVolts();
    public abstract double getCurrent();
}
