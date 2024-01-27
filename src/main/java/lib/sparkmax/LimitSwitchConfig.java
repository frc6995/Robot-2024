package lib.sparkmax;

import java.util.List;

import com.revrobotics.SparkLimitSwitch;

public class LimitSwitchConfig extends Config<SparkLimitSwitch, LimitSwitchConfig> {
    public boolean enabled = true;
    public SparkLimitSwitch.Type type = SparkLimitSwitch.Type.kNormallyOpen;
    public static final List<Call<SparkLimitSwitch, ?, LimitSwitchConfig>> calls = List.of(
        call(
            SparkLimitSwitch::enableLimitSwitch,
            c->c.enabled
        )
    );
    public LimitSwitchConfig setEnabled(boolean enabled) {
        this.enabled = enabled;
        return this;
    }
    public LimitSwitchConfig setType(SparkLimitSwitch.Type type) {
        this.type = type;
        return this;
    }
    
    
}
