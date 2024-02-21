package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class Interpolation {
    public static final double MAX_DISTANCE = 5.63; // wingline
    public static final double MIN_DISTANCE = 1.4; // slightly back from subwoofer
    public static final InterpolatingDoubleTreeMap PIVOT_MAP = new InterpolatingDoubleTreeMap();
    static {
        PIVOT_MAP.put( Units.inchesToMeters(75), Units.degreesToRadians(180-50));
        // PIVOT_MAP.put(5.63, Units.degreesToRadians(180-17));
        // PIVOT_MAP.put(5.0,Units.degreesToRadians(180-20));
        // PIVOT_MAP.put(4.0, Units.degreesToRadians(180-24));
        // PIVOT_MAP.put(3.00, Units.degreesToRadians(180-31));
        // PIVOT_MAP.put(2.00, Units.degreesToRadians(180-41));
        // PIVOT_MAP.put(1.4, Units.degreesToRadians(180-53));
    }

    public static final double dThetadX(double x) {
        var thetaPre = PIVOT_MAP.get(x-0.001);
        var thetaPost = PIVOT_MAP.get(x+0.001);
        return (thetaPost - thetaPre) / 0.002;
    }

    public static final InterpolatingDoubleTreeMap TOP_MAP = new InterpolatingDoubleTreeMap();
    static {
        TOP_MAP.put(Units.inchesToMeters(75), 5764.0);
    }
}
