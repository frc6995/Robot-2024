package lib.sparkmax;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;

public class AbsoluteEncoderConfig extends Config<AbsoluteEncoder, AbsoluteEncoderConfig>  {
    public double zeroOffset = 42;
    public int averageDepth = 8;

    public boolean inverted;
    public double positionConversionFactor = 1;
    public double velocityConversionFactor = 1;
    public static final List<Call<AbsoluteEncoder, ?, AbsoluteEncoderConfig>> calls = List.of(
        call(
            AbsoluteEncoder::setInverted,
            c->c.inverted
        ),
        call(
            AbsoluteEncoder::setPositionConversionFactor,
            c->c.positionConversionFactor
        ),
        call(
            AbsoluteEncoder::setVelocityConversionFactor,
            c->c.velocityConversionFactor
        ),
        call(
            AbsoluteEncoder::setZeroOffset,
            c->c.zeroOffset
        )
    );
    public AbsoluteEncoderConfig zeroOffset(double zeroOffset) {
        this.zeroOffset = zeroOffset;
        return this;
    }
    public AbsoluteEncoderConfig setAverageDepth(int averageDepth) {
        this.averageDepth = averageDepth;
        return this;
    }
    public AbsoluteEncoderConfig inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }
    public AbsoluteEncoderConfig positionConversionFactor(double positionConversionFactor) {
        this.positionConversionFactor = positionConversionFactor;
        return this;
    }
    public AbsoluteEncoderConfig velocityConversionFactor(double velocityConversionFactor) {
        this.velocityConversionFactor = velocityConversionFactor;
        return this;
    }
}