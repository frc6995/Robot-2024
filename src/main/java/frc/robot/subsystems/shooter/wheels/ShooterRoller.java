package frc.robot.subsystems.shooter.wheels;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import monologue.Logged;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

public class ShooterRoller implements Subsystem, Logged {
    private ShooterRollerIO m_io;

    private int canId;
    private String name;
    public final SysIdRoutine m_idRoutine;
    public ShooterRoller(int canId, boolean invert, String name){
        if(RobotBase.isReal()) {
            m_io = new RealShooterRollerIO(canId, invert);
        } else {
            m_io = new SimShooterRollerIO();
        }
        this.name = name;
        MutableMeasure<Velocity<Angle>> velocityMeasure = MutableMeasure.ofBaseUnits(0, RPM);
        MutableMeasure<Voltage> voltsMeasure = MutableMeasure.ofBaseUnits(0, Volts);
        String sysidName = "shooter" + this.name;
        m_idRoutine = new SysIdRoutine(
        new Config(
        Volts.of(0.5).per(Second),
        Volts.of(1),
        Seconds.of(10)
        ), 
        new Mechanism(
        (Measure<Voltage> volts)->m_io.setVolts(volts.in(Volts)),
        (log)->{
            log.motor(sysidName).angularVelocity(
            velocityMeasure.mut_replace(m_io.getVelocity(), RadiansPerSecond)
            ).voltage(
            voltsMeasure.mut_replace(m_io.getVolts(), Volts)
            );

        }, this, sysidName));
        setDefaultCommand(stopC());
    }

    @Override
    public String getPath() {
        return name;
    }
    public void setSpeed(double rpm) {
        m_io.setPIDFF(rpm, rpm);
    }

    public void setVolts(double voltage){
        m_io.setVolts(voltage);
    }

    public Command stopC() {
        return run(()->setVolts(0));
    }

    public Command spinC(DoubleSupplier rpm) {
        return run(()->setSpeed(rpm.getAsDouble()));
    }
}
