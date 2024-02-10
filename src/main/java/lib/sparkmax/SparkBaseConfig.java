package lib.sparkmax;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lib.sparkmax.PIDControllerConfig.FeedbackDevice;

public class SparkBaseConfig extends Config<CANSparkBase, SparkBaseConfig> {
    
    static AbsoluteEncoderConfig absEncDefaults = new AbsoluteEncoderConfig();
    static RelativeEncoderConfig hallEncDefaults = new RelativeEncoderConfig();
    static RelativeEncoderConfig altEncDefaults = new RelativeEncoderConfig();
    static PIDControllerConfig pidDefaults = PIDControllerConfig.defaults;
    static {
        altEncDefaults.averageDepth = 64;
        altEncDefaults.countsPerRev = 4096;
    }
    static LimitSwitchConfig forwardSwitchDefaults = new LimitSwitchConfig();
    static LimitSwitchConfig backwardSwitchDefaults = new LimitSwitchConfig();
    static SparkBaseConfig defaults = new SparkBaseConfig();
    public boolean isInverted = false;
    public float forwardSoftLimit = 0;
    public boolean forwardSoftLimitEnabled = false;
    public float reverseSoftLimit = 0;
    public boolean reverseSoftLimitEnabled = false;
    public int stallLimit = 80;
    public int freeLimit = 20;
    public double nominalVoltage = 0;
    public boolean voltageCompensationEnabled = false;
    public int followerID = -1;
    public boolean followerInvert = false;
    public IdleMode idleMode = IdleMode.kCoast;
    public int status0 = 10;
    public int status1 = 20;
    public int status2 = 20;
    public int status3 = 50;
    public int status4 = 20;
    public int status5 = 200;
    public int status6 = 200;

    public SparkRelativeEncoder.Type encoderPortType = SparkRelativeEncoder.Type.kHallSensor;
    /**
     * This is only used for Spark MAX. For Spark Flex, the altEncoder config will apply 
     * to the External Encoder, and the encoderPortEncoder and limit switches will be configured
     * as well.
     */
    public boolean alternateEncoderMode = false;
    public RelativeEncoderConfig hallEncoder;
    public RelativeEncoderConfig altEncoder;
    public AbsoluteEncoderConfig absEncoder;
    public LimitSwitchConfig forwardSwitch;
    public LimitSwitchConfig backwardSwitch;
    public PIDControllerConfig pid;
    static List<Call<CANSparkBase, ?, SparkBaseConfig>> statusFrameCalls = List.of(
        call((s, s0) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus0, s0), c->c.status0),
        call((s, s1) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus1, s1), c->c.status1),
        call((s, s2) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus2, s2), c->c.status2),
        call((s, s3) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus3, s3), c->c.status3),
        call((s, s4) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus4, s4), c->c.status4),
        call((s, s5) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus5, s5), c->c.status5),
        call((s, s6) -> s.setPeriodicFramePeriod(PeriodicFrame.kStatus6, s6), c->c.status6)
    );
    static List<Call<CANSparkBase, ?, SparkBaseConfig>> calls = List.of(
                // Inversion
                call(
                        (s, inv) -> {
                            s.setInverted(inv);
                            return s.getLastError();
                        },
                        c -> c.isInverted),
                // Forward soft limit
                call(
                        (s, lim) -> s.setSoftLimit(SoftLimitDirection.kForward, lim),
                        c -> c.forwardSoftLimit),
                call(
                        (s, en) -> s.enableSoftLimit(SoftLimitDirection.kForward, en),
                        c -> c.forwardSoftLimitEnabled),
                // Backward soft limit
                call(
                        (s, lim) -> s.setSoftLimit(SoftLimitDirection.kReverse, lim),
                        c -> c.reverseSoftLimit),
                call(
                        (s, en) -> s.enableSoftLimit(SoftLimitDirection.kReverse, en),
                        c -> c.reverseSoftLimitEnabled),
                call(
                        (s, mode) -> s.setIdleMode(mode),
                        c -> c.idleMode),
                call(
                        (s, lims) -> s.setSmartCurrentLimit(lims.getFirst(), lims.getSecond()),
                        c -> new Pair<Integer, Integer>(c.stallLimit, c.freeLimit)),
                call(
                        (s, set) -> set.getSecond() ? s.enableVoltageCompensation(set.getFirst())
                                : s.disableVoltageCompensation(),
                        c -> new Pair<Double, Boolean>(c.nominalVoltage, c.voltageCompensationEnabled)),
                call(
                        (s, set) -> s.follow(
                                set.getFirst() >= 0 ? CANSparkBase.ExternalFollower.kFollowerSpark
                                        : CANSparkBase.ExternalFollower.kFollowerDisabled,
                                set.getFirst(), set.getSecond()),
                        c -> new Pair<Integer, Boolean>(c.followerID, c.followerInvert))
        );

    public SparkBaseConfig() {
        hallEncoder = SparkBaseConfig.hallEncDefaults.clone();
        altEncoder = SparkBaseConfig.altEncDefaults.clone();
        absEncoder = SparkBaseConfig.absEncDefaults.clone();
        forwardSwitch = SparkBaseConfig.forwardSwitchDefaults.clone();
        backwardSwitch = SparkBaseConfig.backwardSwitchDefaults.clone();
        pid = pidDefaults.clone();
    }

    public SparkBaseConfig(Consumer<SparkBaseConfig> editDefaults) {
        this();
        editDefaults.accept(this);
    }

    public SparkBaseConfig copy(Consumer<SparkBaseConfig> editDefaults) {
        var clone = this.clone();
        editDefaults.accept(clone);
        return clone;
    }
    public SparkBaseConfig copy() {
        return this.clone();
    }

    @Override
    public SparkBaseConfig clone() {
        var newConfig = (SparkBaseConfig) super.clone();
        newConfig.hallEncoder = hallEncoder.clone();
        newConfig.altEncoder = altEncoder.clone();
        newConfig.absEncoder = absEncoder.clone();
        newConfig.forwardSwitch = forwardSwitch.clone();
        newConfig.backwardSwitch = backwardSwitch.clone();
        newConfig.pid = pid.clone();
        return newConfig;
    }

    public CANSparkFlex applyFlex(CANSparkFlex s, boolean restoreFactoryDefaults) {
        s = apply(s, restoreFactoryDefaults);
        try {
            // var encoder = s.getExternalEncoder(SparkFlexExternalEncoder.Type.kQuadrature, altEncoder.countsPerRev);
            // altEncoder.apply(encoder, RelativeEncoderConfig.calls, altEncDefaults, restoreFactoryDefaults);
            // if (pid.feedbackSensor == FeedbackDevice.kAlternateEncoder) {
            //     var controller = s.getPIDController();
            //     SparkBaseConfig.config(()->controller.setFeedbackDevice(encoder));
            // }
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
            
        try {
            applyAbsEncAndSwitches(s, restoreFactoryDefaults);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
        Timer.delay(0.2);
        return s;
    }
    public CANSparkMax applyMax(CANSparkMax s, boolean restoreFactoryDefaults) {
        s = apply(s, restoreFactoryDefaults);
        if (alternateEncoderMode) {
            try {
                var alternateEncoder = s.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, altEncoder.countsPerRev);
                altEncoder.apply(alternateEncoder, RelativeEncoderConfig.calls, altEncDefaults, restoreFactoryDefaults);
                
                if (pid.feedbackSensor == FeedbackDevice.kAlternateEncoder) {
                    var controller = s.getPIDController();
                    SparkBaseConfig.config(()->controller.setFeedbackDevice(alternateEncoder));
                }
            } catch (Exception e) {
                DriverStation.reportError(e.getMessage(), e.getStackTrace());
            }
        } else {
            // limit switches + absolute encoder only available in normal mode
            s = applyAbsEncAndSwitches(s, restoreFactoryDefaults);
        }
        Timer.delay(0.2);
        return s;
    }

    /**
     * Applies a configuration to a provided CANSparkBase. 
     * 
     * The purpose of this is for compatibility with wrappers around CANSparkBase.
     * If you are not using a wrapper, consider create(int id, MotorType type, restoreFactoryDefaults)
     * IMPORTANT: The passed-in object should be unconfigured, 
     * and preferably constructed in this method's parameter list.
     * Otherwise, configurations applied here may fail, since some 
     * configurations (particularly around encoders, analog sensors,
     * and limit switches) can only be applied once.
     * 
     * 
     * @param s
     * @param restoreFactoryDefaults
     * @return
     */
    public <S extends CANSparkBase> S apply(S s, boolean restoreFactoryDefaults) {
        if (restoreFactoryDefaults)
            config(s::restoreFactoryDefaults);
        for (Call<CANSparkBase, ?, SparkBaseConfig> config : calls) {
            applyConfig(s, this, defaults, config, restoreFactoryDefaults);
        }
        for (Call<CANSparkBase, ?, SparkBaseConfig> config : statusFrameCalls) {
            applyConfig(s, this, defaults, config, restoreFactoryDefaults);
        }

        // Configure the encoder port;
        if (encoderPortType != SparkRelativeEncoder.Type.kNoSensor) {
            try {
                var mainEncoder = s.getEncoder(encoderPortType, hallEncoder.countsPerRev);
                hallEncoder.apply(mainEncoder, RelativeEncoderConfig.calls, hallEncDefaults, restoreFactoryDefaults);
                if (pid.feedbackSensor == FeedbackDevice.kHallSensor) {
                    SparkBaseConfig.config(()->s.getPIDController().setFeedbackDevice(mainEncoder));
                }
            } catch (Exception e) {
                DriverStation.reportError(e.getMessage(), e.getStackTrace());
            }     
        }
        // Configure the PID controller
        try {
            var pidController = s.getPIDController();
            pid.apply(pidController, PIDControllerConfig.calls, pidDefaults, restoreFactoryDefaults);

        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
        return s;
    }

    public <S extends CANSparkBase> S applyAbsEncAndSwitches(S s, boolean restoreFactoryDefaults) {
        try {
            var encoder = s.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            absEncoder.apply(encoder, AbsoluteEncoderConfig.calls, absEncDefaults, restoreFactoryDefaults);
            var forward = s.getForwardLimitSwitch(forwardSwitch.type);
            forwardSwitch.apply(forward, LimitSwitchConfig.calls, forwardSwitchDefaults, restoreFactoryDefaults);
            var backward = s.getReverseLimitSwitch(forwardSwitch.type);
            backwardSwitch.apply(backward, LimitSwitchConfig.calls, backwardSwitchDefaults, restoreFactoryDefaults);
            if (pid.feedbackSensor == FeedbackDevice.kAbsoluteEncoder) {
                SparkBaseConfig.config(()->s.getPIDController().setFeedbackDevice(encoder));
            }
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
        return s;
    }

    public CANSparkMax createMax(int id, MotorType type, boolean restoreFactoryDefaults) {
        return applyMax(new CANSparkMax(id, type), restoreFactoryDefaults);
    }

    public CANSparkFlex createFlex(int id, MotorType type, boolean restoreFactoryDefaults) {
        return applyFlex(new CANSparkFlex(id, type), restoreFactoryDefaults);
    }


    /**
     * Calls the provided method up to 4 times, repeating if the returned REVLibError is not kOk;
     * @param configCall
     */
    public static boolean config(Supplier<REVLibError> configCall) {
        try {
        int attempts = 0;
        REVLibError error = REVLibError.kOk;
        do {
            if (attempts > 0) {
                Timer.delay(0.010);
            }
            error = configCall.get();
            attempts++;
        } while ((error == REVLibError.kTimeout) && attempts <= 4);
        if (error != REVLibError.kOk) {
            
            DriverStation.reportError("Error configuring", false);
        }} catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            DataLogManager.log(e.getMessage());
        }
        return true;
    }

    /**
     * Returns whether or not the supplied configuration has any changes from this one
     * that could not be updated at runtime
     * @return
     */
    public boolean safeToUpdate(SparkBaseConfig newConfig) {
        return 
            encoderPortType == newConfig.encoderPortType &&
            hallEncoder.countsPerRev == newConfig.hallEncoder.countsPerRev &&
            alternateEncoderMode == newConfig.alternateEncoderMode &&
            (alternateEncoderMode ? (
                altEncoder.countsPerRev == newConfig.altEncoder.countsPerRev
            ) : (true
                // limit switches
            ))

        ;
    }

    public SparkBaseConfig inverted(boolean inverted) {
        this.isInverted = inverted;
        return this;
    }

    public SparkBaseConfig setForwardSoftLimit(float forwardSoftLimit) {
        this.forwardSoftLimit = forwardSoftLimit;
        return this;
    }

    public SparkBaseConfig enableForwardSoftLimit(boolean forwardSoftLimitEnabled) {
        this.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
        return this;
    }

    public SparkBaseConfig setReverseSoftLimit(float reverseSoftLimit) {
        this.reverseSoftLimit = reverseSoftLimit;
        return this;
    }

    public SparkBaseConfig enableReverseSoftLimit(boolean reverseSoftLimitEnabled) {
        this.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
        return this;
    }

    public SparkBaseConfig stallLimit(int stallLimit) {
        this.stallLimit = stallLimit;
        return this;
    }

    public SparkBaseConfig freeLimit(int freeLimit) {
        this.freeLimit = freeLimit;
        return this;
    }

    public SparkBaseConfig disableVoltageCompensation() {
        this.voltageCompensationEnabled = false;
        return this;
    }

    public SparkBaseConfig enableVoltageCompensation(double nominalVoltage) {
        this.voltageCompensationEnabled = true;
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SparkBaseConfig follow(int followerID, boolean followerInvert) {
        this.followerID = followerID;
        this.followerInvert = followerInvert;
        return this;
    }

    public SparkBaseConfig idleMode(IdleMode idleMode) {
        this.idleMode = idleMode;
        return this;
    }

    public SparkBaseConfig setEncoderPortType(SparkRelativeEncoder.Type encoderPortType) {
        this.encoderPortType = encoderPortType;
        return this;
    }

    public SparkBaseConfig setAlternateEncoderMode(boolean alternateEncoderMode) {
        this.alternateEncoderMode = alternateEncoderMode;
        return this;
    }

    public SparkBaseConfig status0(int status0) {
        this.status0 = status0;
        return this;
    }

    public SparkBaseConfig status1(int status1) {
        this.status1 = status1;
        return this;
    }

    public SparkBaseConfig status2(int status2) {
        this.status2 = status2;
        return this;
    }

    public SparkBaseConfig status3(int status3) {
        this.status3 = status3;
        return this;
    }

    public SparkBaseConfig status4(int status4) {
        this.status4 = status4;
        return this;
    }

    public SparkBaseConfig status5(int status5) {
        this.status5 = status5;
        return this;
    }

    public SparkBaseConfig status6(int status6) {
        this.status6 = status6;
        return this;
    }

    public SparkBaseConfig statusFrames(int status0, int status1, int status2, int status3, int status4, int status5, int status6) {
        this.status0 = status0;
        this.status1 = status1;
        this.status2 = status2;
        this.status3 = status3;
        this.status4 = status4;
        this.status5 = status5;
        this.status6 = status6;
        return this;
    }
    
    
    
}
