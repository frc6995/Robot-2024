package frc.robot.subsystems.shooter.wheels;

public abstract class ShooterRollerIO {
    public ShooterRollerIO(){}
    public abstract double getVelocity();
    public abstract void setPIDFF(double velocity, double ffVolts);
    public abstract void periodic();
    public abstract void setVolts(double volts);
    public abstract double getVolts();
    public abstract double getPidVolts();
}
