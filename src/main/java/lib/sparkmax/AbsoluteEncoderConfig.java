package lib.sparkmax;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;

public class AbsoluteEncoderConfig extends Config<AbsoluteEncoder, AbsoluteEncoderConfig>  {
    public double zeroOffset = 0;
    public int averageDepth = 8;

    public boolean inverted = false;
    public double positionConversionFactor = 1;
    public double velocityConversionFactor = 1;
    // These need to be applied in this order so that the zero offset
    // is in the converted units and not native;
    public static final List<Call<AbsoluteEncoder, ?, AbsoluteEncoderConfig>> calls = List.of(
        call(
            AbsoluteEncoder::setInverted,
            c->c.inverted,
            "absEncInv"
        ),
        call(
            AbsoluteEncoder::setPositionConversionFactor,
            c->c.positionConversionFactor,
            "absPosFac"
        ),
        call(
            AbsoluteEncoder::setVelocityConversionFactor,
            c->c.velocityConversionFactor,
            "absVelFac"
        ),
        call(
            AbsoluteEncoder::setZeroOffset,
            c->c.zeroOffset,
            "0off"
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