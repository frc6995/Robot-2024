package frc.robot.subsystems.shooter.wheels;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.sparkmax.SparkDevice;
import lib.sparkmax.SparkBaseConfig;

public class CTREShooterRollerIO extends ShooterRollerIO {

    private TalonFX m_motor;
    private double ffVolts = 0;

    class Constants {
        public static final double SENSOR_TO_MECHANISM_RATIO = 26.0/43.0;
        public static final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                
                .withNeutralMode(NeutralModeValue.Coast)
            )
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            ).withSlot0(
                new Slot0Configs()
                .withKP(0.001)
                .withKS(0.00)
                .withKV(12 / 9000.0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(60)
            )
        ;
    }

    public final SysIdRoutine sysIdRoutine;
    public final VoltageOut sysIdControl = new VoltageOut(0);
    public final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private LinearSystemSim<N2, N1, N2> m_sim;
    public CTREShooterRollerIO(int CAN_ID, boolean invert, Slot0Configs pidConfigs, ShooterRoller self) {
        m_motor = new TalonFX(CAN_ID);
        Constants.config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        var configurator = m_motor.getConfigurator();
        configurator.apply(Constants.config);
        configurator.apply(pidConfigs);

        sysIdRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,
			Volts.of(7),
			null,
			(state) -> SignalLogger.writeString("state-"+CAN_ID, state.toString())),
		new SysIdRoutine.Mechanism(
			(volts) -> m_motor.setControl(sysIdControl.withOutput(volts.baseUnitMagnitude())),
			null,
			self));
        m_sim = new LinearSystemSim<N2, N1, N2>(LinearSystemId.createDCMotorSystem(pidConfigs.kV, pidConfigs.kA));

    }

    @Override
    public double getVelocity() {
        return m_motor.getVelocity().getValue() * 60;
    }

    @Override
    public void setPIDFF(double velocityRPM, double ffVolts) {
        this.ffVolts = ffVolts;
        m_motor.setControl(new VelocityVoltage(velocityRPM / 60.0));
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            for (int i = 0; i < 5; i++) {
            var simState = m_motor.getSimState();
            simState.setSupplyVoltage(12);
            double volts = simState.getMotorVoltage();
            m_sim.setInput(volts);
            m_sim.update(0.004);
            simState.setRawRotorPosition(m_sim.getOutput(0)* Constants.SENSOR_TO_MECHANISM_RATIO);
            simState.setRotorVelocity(m_sim.getOutput(1) * Constants.SENSOR_TO_MECHANISM_RATIO);
            }
        }
        
    }

    @Override
    public void setVolts(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public double getVolts() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public double getPidVolts() {
        return getVolts() - ffVolts;
    }

    public double getCurrent() {
        return m_motor.getSupplyCurrent().getValue();
    }
    @Override
    public double getPosition() {
        return m_motor.getPosition().getValue();
    }


    
}
