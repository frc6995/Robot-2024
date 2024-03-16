package frc.robot.subsystems.bounceBar;

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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;

public class SimBounceBarIO extends BounceBarIO {
    private double m_pidVolts = 0;
    private PIDController m_pid = new PIDController(50.219, 0, 3.0547);
    public SimBounceBarIO () {
        super();
        m_pivotSim.setState(VecBuilder.fill(BounceBarS.Constants.CCW_LIMIT,0));
        // we need this to calculate outputs
        m_pivotSim.update(0.0001);

    }
    @Override
    public double getAngle() {
        return m_pivotSim.getAngleRads();
    }
    @Override
    public double getVelocity() {
        return m_pivotSim.getVelocityRadPerSec();
    }

    @Override
    public void setPIDFF(double angle, double ffVolts) {
        m_pidVolts = m_pid.calculate(getAngle(), angle);
        setVolts(m_pidVolts + ffVolts);
    }
    @Override
    public double getPidVolts() {
        return m_pidVolts;
    }
    public void resetAngle(double angle) {
        m_pivotSim.setState(angle, 0);
    }

    private final SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(
        Constants.PLANT,
        DCMotor.getNEO(1),
        BounceBarS.Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION,
        BounceBarS.Constants.CG_DIST * 2,
        BounceBarS.Constants.CW_LIMIT,
        BounceBarS.Constants.CCW_LIMIT,
        false,
        BounceBarS.Constants.CCW_LIMIT,
        null
    );

    private double m_inputVolts;
    
    public void setVolts(double volts) {
        volts = MathUtil.clamp(DriverStation.isEnabled() ? volts : 0, -12, 12);
        m_inputVolts = NomadMathUtil.subtractkS(volts, BounceBarS.Constants.K_S);
        m_pivotSim.setInputVoltage(m_inputVolts - BounceBarS.Constants.K_G * Math.cos(getAngle())); 
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            m_pivotSim.setInputVoltage(0);
        }
        m_pivotSim.update(TimingTracer.getLoopTime());
    }

    @Override
    public double getCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }
    public double getVolts() {
        return m_inputVolts;
    }

    public class Constants {
        public static final LinearSystem<N2, N1, N1> PLANT =
            LinearSystemId.identifyPositionSystem(BounceBarS.Constants.K_V, BounceBarS.Constants.K_A);
    }    
}
