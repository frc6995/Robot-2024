package frc.robot.subsystems.shooter.wheels;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.logging.RevFaults;
import monologue.Annotations.Log;
import monologue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;

public class ShooterRoller implements Subsystem, Logged {
    private CTREShooterRollerIO m_io;

    public final Trigger atGoal = new Trigger(this::atGoal);

    private int canId;
    private String name;
    public final SysIdRoutine m_idRoutine;
    private SimpleMotorFeedforward ff;
    private double desiredSpeed;
    public ShooterRoller(int canId, boolean invert, double kS, double kV, double kA, double kP, String name){

        m_io = new CTREShooterRollerIO(canId, invert,
            new Slot0Configs().withKS(kS).withKV(kV).withKA(kA).withKP(kP),
            new Slot1Configs().withKS(10).withKV(0).withKA(0).withKP(10)
            , this);
        m_idRoutine = m_io.sysIdRoutine;
        // if(RobotBase.isReal()) {
        //     m_io = new RealShooterRollerIO(canId, invert);
        // } else {
        //     m_io = new SimShooterRollerIO(kS, kV);
        // }
        this.ff = new SimpleMotorFeedforward(kS, kV);
        this.name = name;
        setDefaultCommand(stopC());
    }
    @Log public double getGoalVelocity() {return desiredSpeed;}
    @Log public double getVelocity() {return m_io.getVelocity();}
    @Log public double getPidVolts() {return m_io.getPidVolts();}
    @Log public double getVolts() {return m_io.getVolts();}
    @Log public double getCurrent() {return m_io.getCurrent();}
    @Log public double getPosition() {return m_io.getPosition();}

    public boolean atGoal() {
        return Math.abs(desiredSpeed - getVelocity()) < 50;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setVolts(0);
        }
        m_io.periodic();
    }
    @Override
    public String getPath() {
        return name;
    }
    public void setSpeed(double rpm) {
        desiredSpeed = rpm;
        m_io.setPIDFF(rpm, ff.calculate(rpm));
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
    public Command voltsC(DoubleSupplier volts) {
        return run(()->setVolts(volts.getAsDouble()));
    }
}
