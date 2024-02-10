package lib.sparkmax;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class RelativeEncoderConfig extends Config<RelativeEncoder, RelativeEncoderConfig> {
    public int countsPerRev = 42;
    public int averageDepth = 8;
    public boolean isInverted;
    public int measurementPeriod = 32;
    public double positionConversionFactor = 1;
    public double velocityConversionFactor = 1;
    public final static List<Call<RelativeEncoder, ?, RelativeEncoderConfig>> calls = List.of(
        call(
            RelativeEncoder::setInverted,
            c->c.isInverted
        ),
        call(
            RelativeEncoder::setPositionConversionFactor,
            c->c.positionConversionFactor
        ),
        call(
            RelativeEncoder::setVelocityConversionFactor,
            c->c.velocityConversionFactor
        ),
        call(
            RelativeEncoder::setMeasurementPeriod,
            c->c.measurementPeriod
        ),
        call(
            RelativeEncoder::setAverageDepth,
            c->c.averageDepth
        )
    );
    public RelativeEncoderConfig countsPerRev(int countsPerRev) {
        this.countsPerRev = countsPerRev;
        return this;
    }
    public RelativeEncoderConfig averageDepth(int averageDepth) {
        this.averageDepth = averageDepth;
        return this;
    }
    public RelativeEncoderConfig inverted(boolean inverted) {
        this.isInverted = inverted;
        return this;
    }
    public RelativeEncoderConfig measurementPeriod(int measurementPeriod) {
        this.measurementPeriod = measurementPeriod;
        return this;
    }
    public RelativeEncoderConfig positionConversionFactor(double positionConversionFactor) {
        this.positionConversionFactor = positionConversionFactor;
        return this;
    }
    public RelativeEncoderConfig velocityConversionFactor(double velocityConversionFactor) {
        this.velocityConversionFactor = velocityConversionFactor;
        return this;
    }
    
}
