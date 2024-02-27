package frc.robot.subsystems.shooter.wheels;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;

public class SimShooterRollerIO extends ShooterRollerIO {

    private PIDController m_controller = new PIDController(0, 0, 0);

    private FlywheelSim m_sim;
    private double inputVolts = 0;
    private double pidVolts = 0;

    public SimShooterRollerIO(double kS, double kV) {
        super();
        m_sim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Units.radiansPerSecondToRotationsPerMinute(kV), 0.001), DCMotor.getNeoVortex(1), 1);
    }
    @Override
    public double getVelocity() {
        return m_sim.getAngularVelocityRPM();
    }

    @Override
    public void setPIDFF(double velocity, double ffVolts) {
        pidVolts = m_controller.calculate(getVelocity(), velocity);
        inputVolts = pidVolts + ffVolts;
        setVolts(inputVolts);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setVolts(0);
        }
        m_sim.update(TimingTracer.getLoopTime());
    }

    @Override
    public void setVolts(double volts) {
        m_sim.setInputVoltage(NomadMathUtil.subtractkS(volts, 0.1));
    }

    @Override
    public double getVolts() {
        return inputVolts;
    }

    @Override
    public double getPidVolts() {
        return pidVolts;
    }

    public double getCurrent() {
        return 0;
    }

    @Override
    public double getPosition() {
        return 0;
    }
    
}
