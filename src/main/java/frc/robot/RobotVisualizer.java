package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
    private static final double BASE_X = Units.feetToMeters(3);
    public static final Mechanism2d MECH_VISUALIZER = new Mechanism2d(BASE_X * 2, Units.feetToMeters(5));
    private static final MechanismRoot2d MECH_VISUALIZER_ROOT = MECH_VISUALIZER.getRoot("root", BASE_X, 0);
    private static final MechanismRoot2d SHOOTER_PIVOT_BASE = MECH_VISUALIZER.getRoot("shooter-pivot-base", BASE_X - Units.inchesToMeters(4), Units.inchesToMeters(9));
    private static final MechanismLigament2d BACK_DRIVETRAIN_HALF = new MechanismLigament2d(
        "drive-front", Units.inchesToMeters(12.5), 180);
    public static void setupVisualizer() {
        MECH_VISUALIZER_ROOT.append(BACK_DRIVETRAIN_HALF);
    }
    public static void addShooter(MechanismLigament2d shooter) {
        SHOOTER_PIVOT_BASE.append(shooter);
    }

}
