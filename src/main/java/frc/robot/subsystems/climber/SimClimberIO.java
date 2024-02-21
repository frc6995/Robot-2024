package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;

public class SimClimberIO extends ClimberIO {
    private double m_pidVolts = 0;
    private PIDController m_pid = new PIDController(1, 0, 0);
    public SimClimberIO (boolean isLeft) {
        super(isLeft);
        m_elevatorSim.setState(VecBuilder.fill(ClimberS.Constants.LOWER_LIMIT,0));
        // we need this to calculate outputs
        m_elevatorSim.update(0.0001);

    }
    @Override
    public double getLength() {
        return m_elevatorSim.getPositionMeters();
    }

    @Override
    public void setPIDFF(double length, double ffVolts) {
        m_pidVolts = m_pid.calculate(getLength(), length);
        setVolts(m_pidVolts + ffVolts);
    }
    @Override
    public double getPidVolts() {
        return m_pidVolts;
    }
    public void resetLength(double length) {
        m_elevatorSim.setState(length, 0);
    }

    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        Constants.PLANT,
        DCMotor.getNeoVortex(1),
        ClimberS.Constants.LOWER_LIMIT,
        ClimberS.Constants.UPPER_LIMIT,
        true,
        ClimberS.Constants.LOWER_LIMIT
    );

    private double m_inputVolts;
    
    public void setVolts(double volts) {
        volts = MathUtil.clamp(DriverStation.isEnabled() ? volts : 0, -12, 12);
        m_inputVolts = NomadMathUtil.subtractkS(volts, 0);
        m_elevatorSim.setInputVoltage(m_inputVolts); 
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            m_elevatorSim.setInputVoltage(0);
        }
        m_elevatorSim.update(TimingTracer.getLoopTime());
    }

    public double getVolts() {
        return m_inputVolts;
    }

    public class Constants {
        public static final LinearSystem<N2, N1, N1> PLANT =
            LinearSystemId.createElevatorSystem(DCMotor.getNeoVortex(1), 1, 1.0/(2*Math.PI),ClimberS.Constants.MOTOR_ROTATIONS_PER_METER);
    }
    
}
