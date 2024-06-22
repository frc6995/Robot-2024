package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.pivot.ShooterPivotS.Constants.CW_LIMIT;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class Interpolation {
    public static final double AMP_SPEED = 4000.0;
    public static final double AMP_PIVOT = CW_LIMIT+ Units.degreesToRadians(0);
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
    // Orange County
    // static {
    //     entry(1.45, CW_LIMIT, 7000,7000);        
    //     entry(1.5, CW_LIMIT, 7000, 7000);
    //     entry(1.75, 2.336-Units.degreesToRadians(1), 7000, 7000);
    //     entry(2, 2.4 - Units.degreesToRadians(2), 7000, 7000);
    //     entry(2.5, 2.48-Units.degreesToRadians(2), 7000, 7000);
    //     entry(2.75, 2.51-Units.degreesToRadians(3), 7000, 7000);
    //     entry(3.1, 2.56- Units.degreesToRadians(3), 7000, 7000);
    //     entry(3.5, 2.6-Units.degreesToRadians(4), 7500, 7500);
    //     entry(4, 2.61-Units.degreesToRadians(2), 8000, 8000);
    //     entry(4.5, 2.63-Units.degreesToRadians(3), 9000, 9000);
    //     entry(5, 2.65, 9000, 9000);
    //     entry(5.447, 2.66, 9000, 9000);
    // }
    // Workshop
    static {
        var five = Units.degreesToRadians(5);
        //cw limit 2.216
        entry(1.45, CW_LIMIT, 7000,7000);      
        entry(1.5, CW_LIMIT, 7000, 7000);
        entry(1.75, 2.336, 7000, 7000);
        entry(2, 2.4, 7000, 7000);
        entry(2.5, 2.48, 7000, 7000);
        entry(2.75, 2.51, 7000, 7000);
        entry(3.1, 2.56, 7000, 7000);
        entry(3.5, 2.6, 7500, 7500);
        entry(3.7, 2.6-Units.degreesToRadians(0.5), 7500, 7500);
        entry(4, 2.6-Units.degreesToRadians(0.5), 8000, 8000);
        entry(4.5, 2.63, 9000, 9000);
        entry(5, 2.65, 9000, 9000);
        entry(5.447, 2.66, 9000, 9000);
    }

    public static final double dThetadX(double x) {
        var thetaPre = PIVOT_MAP.get(x-0.001);
        var thetaPost = PIVOT_MAP.get(x+0.001);
        return (thetaPost - thetaPre) / 0.002;
    }
}
