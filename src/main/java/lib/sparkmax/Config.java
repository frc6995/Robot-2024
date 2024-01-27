package lib.sparkmax;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

/**
 * P = Revlib class
 * E = Config<P,E>
 */
public abstract class Config <P, E extends Config<P,E>> implements Cloneable{
    public record Call<P, T, E>(BiFunction<P, T, REVLibError> setter, Function<E, T> getChangedValue) {}

    public <T> void applyConfig(P e, E config, E defaults, Call<P, T, E> call, boolean restoreFactoryDefaults) {
        T desiredResult = call.getChangedValue().apply(config);
        T defaultVal = call.getChangedValue().apply(defaults);
        if (!restoreFactoryDefaults ||
                (restoreFactoryDefaults && desiredResult != defaultVal)) {
            SparkBaseConfig.config(()->call.setter().apply(e, desiredResult));
        }
    }

    public void apply(P e, List<Call<P, ?, E>> calls, Config<P,E> defaults, boolean restoreFactoryDefaults) {
            for (Call<P, ?, E> config : calls) {
                applyConfig(e, (E) this, (E) defaults, config, restoreFactoryDefaults);
            }
    }

    protected static <P, T, E> Call<P, T, E> call(BiFunction<P, T, REVLibError> setter, Function<E, T> getChangedValue) {
        return new Call<P, T, E>(setter, getChangedValue);
    }

    @Override
    public E clone() {
        try {
            return (E) super.clone();
        } catch (CloneNotSupportedException e) {
            // this implements cloneable and is direct subclass of Object, so this catch block never hits;
            throw new AssertionError("A Config object was not cloneable, but they all should be!");
        }
   }
}
