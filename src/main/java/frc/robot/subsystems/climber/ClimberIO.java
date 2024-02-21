package frc.robot.subsystems.climber;

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
}
