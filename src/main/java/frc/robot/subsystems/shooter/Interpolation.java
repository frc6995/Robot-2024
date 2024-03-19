package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.pivot.ShooterPivotS.Constants.CW_LIMIT;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class Interpolation {
    public static final double AMP_SPEED = 4000.0;
    public static final double AMP_PIVOT = CW_LIMIT+ Units.degreesToRadians(1);
    public static final double MAX_DISTANCE = 4;
    public static final double MIN_DISTANCE = 1;
    public static final InterpolatingDoubleTreeMap PIVOT_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap TOP_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap BOTTOM_MAP = new InterpolatingDoubleTreeMap();
    public static final void entry (double distance, double pivot, double top, double bottom) {
        PIVOT_MAP.put(distance, pivot);
        TOP_MAP.put(distance, top);
        BOTTOM_MAP.put(distance, bottom);
    }
    static {
        var five = Units.degreesToRadians(5);
        entry(1.45, CW_LIMIT, 7000,7000);
        // entry(2.100, 2.333, 5700, 6000);
        // entry(2.652, 2.434, 5600, 6400);
        // entry(3.100, 2.496, 5600, 6400);
        // entry(3.670, 2.580, 5500, 6400);
        // entry(4.200, 2.655, 5500, 6500);
        // entry(4.640, 2.665, 5500, 6500);
        
        entry(1.5, 2.257, 7000, 7000);
        entry(1.75, 2.336, 7000, 7000);
        entry(2, 2.43, 7000, 7000);
        entry(2.5, 2.44, 7000, 7000);
        entry(2.58, 2.46, 7000, 7000);
        entry(2.75, 2.6-five, 7000, 7000);
        entry(3.1, 2.63-five, 7000, 7000);
        entry(3.5, 2.59, 7500, 7500);
        entry(4.1, 2.62, 8000, 8000);
        entry(4.5, 2.63, 9000, 9000);
        entry(5, 2.65, 9000, 9000);
        entry(5.447, 2.66, 9000, 9000);
        // PIVOT_MAP.put(1.45, CW_LIMIT);
        // TOP_MAP.put(1.45, 6000);
        // final var deg = Units.degreesToRadians(1);
        // PIVOT_MAP.put(1.574, 2.217-deg);
        // PIVOT_MAP.put(2.114, 2.353-deg);
        // PIVOT_MAP.put(2.592, 2.486-deg);
        // PIVOT_MAP.put(3.067, 2.564-deg);
        // PIVOT_MAP.put(3.567, 2.620-deg);
        // PIVOT_MAP.put(4.076, 2.686-deg);
        // PIVOT_MAP.put(4.600, 2.718-deg);
        // PIVOT_MAP.put(5.115, 2.735-deg);
        // PIVOT_MAP.put( Units.inchesToMeters(75), Units.degreesToRadians(180-50));
        // PIVOT_MAP.put(2.545, 2.408);
        // PIVOT_MAP.put(2.327, 2.366);
        // PIVOT_MAP.put(1.745, 2.289);
        // PIVOT_MAP.put(1.25, CW_LIMIT);
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
}
