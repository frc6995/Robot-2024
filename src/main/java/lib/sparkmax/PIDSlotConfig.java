package lib.sparkmax;

import java.util.List;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

public class PIDSlotConfig extends Config<SparkPIDController, PIDSlotConfig> {
    public double p = 0.0;
    public double i = 0.0;
    public double d = 0.0;
    public double ff = 0.0;
    public static final List<Call<SparkPIDController, ?, PIDSlotConfig>> calls = List.of();

    public static List<Call<SparkPIDController, ?, PIDSlotConfig>> createCallsList(int slot) {
        return List.of(
            call((s, kP)->s.setP(kP, slot), c->c.p),
            call((s, kI)->s.setI(kI, slot), c->c.i),
            call((s, kD)->s.setD(kD, slot), c->c.d),
            call((s, kFF)->s.setFF(kFF, slot), c->c.ff)
        );
    }

    public PIDSlotConfig p(double kP) {
        this.p = kP;
        return this;
    }

    public PIDSlotConfig i(double kI) {
        this.i = kI;
        return this;
    }

    public PIDSlotConfig d(double kD) {
        this.d = kD;
        return this;
    }

    public PIDSlotConfig ff(double kFF) {
        this.ff = kFF;
        return this;
    }
    public PIDSlotConfig pidFF(double p, double i, double d, double ff) {
        return this.p(p).i(i).d(d).ff(ff);
    }
    
}
