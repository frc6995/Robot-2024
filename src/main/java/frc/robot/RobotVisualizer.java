package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
    private static final double BASE_X = Units.feetToMeters(3);
    private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);
    public static final Mechanism2d MECH_VISUALIZER = new Mechanism2d(BASE_X * 2, Units.feetToMeters(5));
    private static final MechanismRoot2d MECH_VISUALIZER_ROOT = MECH_VISUALIZER.getRoot("root", BASE_X, Units.inchesToMeters(7.5));
    private static final MechanismRoot2d SHOOTER_PIVOT_BASE = MECH_VISUALIZER.getRoot("shooter-pivot-base", BASE_X - Units.inchesToMeters(4.5), Units.inchesToMeters(12));
    private static final MechanismRoot2d INTAKE_PIVOT_BASE = MECH_VISUALIZER.getRoot("intake-pivot-base", BASE_X + Units.inchesToMeters(11.5), Units.inchesToMeters(9.5));
    private static final MechanismRoot2d CLIMBER_BASE = MECH_VISUALIZER.getRoot("climber-base", BASE_X, Units.inchesToMeters(3));
    private static final MechanismLigament2d BACK_DRIVETRAIN_HALF = new MechanismLigament2d(
        "drive-back", Units.inchesToMeters(12.5), 180, 4, ORANGE);
    private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF = new MechanismLigament2d(
        "drive-front", Units.inchesToMeters(12.5), 0, 4, ORANGE);
    public static void setupVisualizer() {
        MECH_VISUALIZER_ROOT.append(BACK_DRIVETRAIN_HALF);
        MECH_VISUALIZER_ROOT.append(FRONT_DRIVETRAIN_HALF);
    }
    public static void addShooter(MechanismLigament2d shooter) {
        SHOOTER_PIVOT_BASE.append(shooter);
    }

    public static void addMidtake(MechanismLigament2d midtake) {
        SHOOTER_PIVOT_BASE.append(midtake);
    }
    public static void addIntake(MechanismLigament2d intake) {
        INTAKE_PIVOT_BASE.append(intake);
    }
    public static void addClimber(MechanismLigament2d climber){
        CLIMBER_BASE.append(climber);
    }


}
