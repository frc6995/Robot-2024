package lib.sparkmax;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * P = Revlib class
 * E = Config<P,E>
 */
public abstract class Config <P, E extends Config<P,E>> implements Cloneable{
    public record Call<P, T, E>(BiFunction<P, T, REVLibError> setter, Function<E, T> getChangedValue) {}
    public static Consumer<String> configErrorFeedback= (s)->DriverStation.reportWarning(s, false);
    public static Consumer<String> configProgressFeedback= System.out::println;
    public <T> void applyConfig(P e, E config, E defaults, Call<P, T, E> call, boolean restoreFactoryDefaults) {
        T desiredResult = call.getChangedValue().apply(config);
        T defaultVal = call.getChangedValue().apply(defaults);
        if ((!restoreFactoryDefaults) ||
                (restoreFactoryDefaults && !desiredResult.equals(defaultVal))) {

            SparkBaseConfig.config(()->call.setter().apply(e, desiredResult));
            //Timer.delay(0.1);
        }
    }

    public void apply(P target, List<Call<P, ?, E>> calls, Config<P,E> defaults, boolean restoreFactoryDefaults) {
                for (Call<P, ?, E> config : calls) {
                    try{
                        applyConfig(target, (E) this, (E) defaults, config, restoreFactoryDefaults);
                        } catch (Exception ex) {
                            configErrorFeedback.accept(ex.getMessage());
                        }
                    }
   
    
    }

    protected static <P, T, E> Call<P, T, E> call(BiFunction<P, T, REVLibError> setter, Function<E, T> getChangedValue, String name) {
        return new Call<P, T, E>((revobj, value)->{
            REVLibError error = setter.apply(revobj, value);
            if (error != REVLibError.kOk) {
            configErrorFeedback.accept(name + ": " + error.toString());
            }

            return error;}, getChangedValue);
    }

    // protected static <P, T, E> Call<P, T, E> call(BiFunction<P, T, REVLibError> setter, Function<E, T> getChangedValue) {
    //     return call(setter, getChangedValue, "Unnamed");
    // }

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
