package lib.sparkmax;

import java.util.List;
import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class PIDControllerConfig extends Config<SparkPIDController, PIDControllerConfig> {
    public static enum FeedbackDevice {
        kHallSensor,
        kAbsoluteEncoder,
        kAlternateEncoder;
    }
 
    public PIDSlotConfig slot0 = new PIDSlotConfig();
    public PIDSlotConfig slot1 = new PIDSlotConfig();
    public PIDSlotConfig slot2 = new PIDSlotConfig();
    public PIDSlotConfig slot3 = new PIDSlotConfig();

    public boolean wrappingEnabled = false;
    public double wrappingMinInput = 0;
    public double wrappingMaxInput = 0;
    public FeedbackDevice feedbackSensor = FeedbackDevice.kHallSensor;
    public static final PIDSlotConfig slotDefaults = new PIDSlotConfig();

    public static final PIDControllerConfig defaults = new PIDControllerConfig();
    static {
        defaults.slot0 = slotDefaults;
        defaults.slot1 = slotDefaults;
        defaults.slot2 = slotDefaults;
        defaults.slot3 = slotDefaults;
    }
    public static final List<Call<SparkPIDController, ?, PIDControllerConfig>> calls = List.of(
        call(SparkPIDController::setPositionPIDWrappingEnabled, c->c.wrappingEnabled),
        call(SparkPIDController::setPositionPIDWrappingMinInput, c->c.wrappingMinInput),
        call(SparkPIDController::setPositionPIDWrappingMaxInput, c->c.wrappingMaxInput)
    );
    private static final List<Call<SparkPIDController, ?, PIDSlotConfig>> slot0Calls = PIDSlotConfig.createCallsList(0);
    private static final List<Call<SparkPIDController, ?, PIDSlotConfig>> slot1Calls = PIDSlotConfig.createCallsList(1);
    private static final List<Call<SparkPIDController, ?, PIDSlotConfig>> slot2Calls = PIDSlotConfig.createCallsList(2);
    private static final List<Call<SparkPIDController, ?, PIDSlotConfig>> slot3Calls = PIDSlotConfig.createCallsList(3);

    public void apply(SparkPIDController p, List<Call<SparkPIDController, ?, PIDControllerConfig>> calls, PIDControllerConfig defaults, boolean restoreFactoryDefaults) {
        super.apply(p, calls, defaults, restoreFactoryDefaults);
        slot0.apply(p, slot0Calls, slotDefaults, restoreFactoryDefaults);
        slot1.apply(p, slot1Calls, slotDefaults, restoreFactoryDefaults);
        slot2.apply(p, slot2Calls, slotDefaults, restoreFactoryDefaults);
        slot3.apply(p, slot3Calls, slotDefaults, restoreFactoryDefaults);
    }

    @Override
    public PIDControllerConfig clone() {
        var newConfig = (PIDControllerConfig) super.clone();
        newConfig.slot0 = slot0.clone();
        newConfig.slot1 = slot1.clone();
        newConfig.slot2 = slot2.clone();
        newConfig.slot3 = slot3.clone();
        return newConfig;
    }

    public PIDControllerConfig wrappingEnabled(boolean wrappingEnabled) {
        this.wrappingEnabled = wrappingEnabled;
        return this;
    }

    public PIDControllerConfig wrappingMinInput(double wrappingMinInput) {
        this.wrappingMinInput = wrappingMinInput;
        return this;
    }

    public PIDControllerConfig wrappingMaxInput(double wrappingMaxInput) {
        this.wrappingMaxInput = wrappingMaxInput;
        return this;
    }

    public PIDControllerConfig feedbackSensor(FeedbackDevice feedbackSensor) {
        this.feedbackSensor = feedbackSensor;
        return this;
    }

    public PIDControllerConfig p(double kP) {
        slot0.p(kP);
        return this;
    }

    public PIDControllerConfig i(double kI) {
        slot0.i(kI);
        return this;
    }

    public PIDControllerConfig d(double kD) {
        slot0.d(kD);
        return this;
    }

    public PIDControllerConfig ff(double kFF) {
        slot0.ff(kFF);
        return this;
    }
    public PIDControllerConfig pidFF(double p, double i, double d, double ff) {
        slot0.pidFF(p, i, d, ff);
        return this;
    }
}
