package frc.robot.subsystems.amp.pivot;

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

public class SimAmpPivotIO extends AmpPivotIO {
    private double m_pidVolts = 0;
    private PIDController m_pid = new PIDController(1, 0, 0);
    public SimAmpPivotIO () {
        super();
        m_pivotSim.setState(VecBuilder.fill(AmpPivotS.Constants.CW_LIMIT,0));
        // we need this to calculate outputs
        m_pivotSim.update(0.0001);

    }
    @Override
    public double getAngle() {
        return m_pivotSim.getAngleRads();
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
        AmpPivotS.Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION,
        AmpPivotS.Constants.CG_DIST * 2,
        AmpPivotS.Constants.CW_LIMIT,
        AmpPivotS.Constants.CCW_LIMIT,
        true,
        AmpPivotS.Constants.CW_LIMIT,
        null
    );

    private double m_inputVolts;
    
    public void setVolts(double volts) {
        volts = MathUtil.clamp(DriverStation.isEnabled() ? volts : 0, -12, 12);
        m_inputVolts = NomadMathUtil.subtractkS(volts, 0);
        m_pivotSim.setInputVoltage(m_inputVolts); 
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            m_pivotSim.setInputVoltage(0);
        }
        m_pivotSim.update(TimingTracer.getLoopTime());
    }

    public double getVolts() {
        return m_inputVolts;
    }

    @Override
    public double getCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }

    public class Constants {
        public static final LinearSystem<N2, N1, N1> PLANT =
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1), 
                SingleJointedArmSim.estimateMOI(AmpPivotS.Constants.CG_DIST, 1),
            AmpPivotS.Constants.MOTOR_ROTATIONS_PER_ARM_ROTATION);
    }
    
}
