package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class Constants {

  public static final class DriveConstants {
    public static final double WHEEL_BASE_WIDTH_M = Units.inchesToMeters(18.75);
    public static final double WHEEL_RADIUS_M =
        Units.inchesToMeters(
            4.0 / 2.0); // 0.0508; //Units.inchesToMeters(4.0/2.0); //four inch (diameter) wheels
    public static final double ROBOT_MASS_kg = Units.lbsToKilograms(150);
    public static final double ROBOT_MOI_KGM2 = 5.77; // Model moment of intertia as a square slab slightly bigger than wheelbase with
    // axis through center
    // Drivetrain Performance Mechanical limits
    public static final double MAX_MODULE_SPEED_MPS = 4.708;
    public static final double MAX_FWD_REV_SPEED_MPS = 4.708;
    public static final double MAX_STRAFE_SPEED_MPS = 4.708;
    public static final double MAX_ROTATE_SPEED_RAD_PER_SEC = 11.5;

    // For manual driving
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15);
    public static final double MAX_TURN_SPEED = Units.degreesToRadians(300);
    // HELPER ORGANIZATION CONSTANTS
    public static final int FL = 0; // Front Left Module Index
    public static final int FR = 1; // Front Right Module Index
    public static final int BL = 2; // Back Left Module Index
    public static final int BR = 3; // Back Right Module Index
    public static final int NUM_MODULES = 4;

    // Internal objects used to track where the modules are at relative to
    // the center of the robot, and all the implications that spacing has.
    private static double HW = WHEEL_BASE_WIDTH_M / 2.0;

    public enum ModuleConstants {
      FL("FL", 18, 17, 6, 0, HW, HW),
      FR("FR", 12, 11, 7, 0, HW, -HW),
      BL("BL", 16, 15, 8, 0, -HW, HW),
      BR("BR", 14, 13, 9, 0, -HW, -HW);

      public final String name;
      public final int driveMotorID;
      public final int rotationMotorID;
      public final int magEncoderID;

      /**
       * absolute encoder offsets for the wheels 180 degrees added to offset values to invert one
       * side of the robot so that it doesn't spin in place
       */
      public final double magEncoderOffset;

      public final Translation2d centerOffset;

      private ModuleConstants(
          String name,
          int driveMotorID,
          int rotationMotorID,
          int magEncoderID,
          double magEncoderOffset,
          double xOffset,
          double yOffset) {
        this.name = name;
        this.driveMotorID = driveMotorID;
        this.rotationMotorID = rotationMotorID;
        this.magEncoderID = magEncoderID;
        this.magEncoderOffset = magEncoderOffset;
        centerOffset = new Translation2d(xOffset, yOffset);
      }
    }

    public static final double WHEEL_REVS_PER_ENC_REV = 1.0 / 6.12;
    public static final double AZMTH_REVS_PER_ENC_REV = 7.0 / 150.0;

    public static final double STEER_MAX_SPEED_RAD_PER_SEC = 7.8 * 2 * Math.PI;
    public static final double STEER_MAX_ACCEL_RAD_PER_SEC_SQ = 400 * 2 * Math.PI;

    // kv: (12 volts * 60 s/min * 1/5.14 WRevs/MRevs * wheel rad * 2pi  / (6000 MRPM *
    /** ks, kv, ka */
    //code orange kaAng 0.77219 V/(m/s^2) kaLin 0.80454
    //kvLin 2.2538
    //ka
    public static final double[] DRIVE_FF_CONST = {0.14315, 2, 0.2};//0.80454};
    public static final double DIAG_TW_HALF = 0.336 / 2.0;
    public static final double ANGULAR_DRIVE_KA = 0.77219;

    public static final double moi = 71.618 * (0.336/2.0) * ANGULAR_DRIVE_KA / 0.80454
    ;
    public static final double STEER_P = 2.3584;
    public static final double STEER_D = 0.01;
    // 12 volts / (5676rpm *2pi radPerRev  / 60 spm / 12.8 revsPerWheelRev)
    public static final double STEER_KV = 12.0 / (5676 * (2 * Math.PI) / 60 / 12.8);

    public static final double DRIVE_P = 8; // 9;
    public static final double DRIVE_D = 0.15;

    public static final int ENC_PULSE_PER_REV = 1;
    public static final double WHEEL_ENC_COUNTS_PER_WHEEL_REV =
        ENC_PULSE_PER_REV / WHEEL_REVS_PER_ENC_REV; // Assume 1-1 gearing for now
    public static final double AZMTH_ENC_COUNTS_PER_MODULE_REV =
        ENC_PULSE_PER_REV / AZMTH_REVS_PER_ENC_REV; // Assume 1-1 gearing for now
    public static final double WHEEL_ENC_WHEEL_REVS_PER_COUNT =
        1.0 / ((double) (WHEEL_ENC_COUNTS_PER_WHEEL_REV));
    public static final double AZMTH_ENC_MODULE_REVS_PER_COUNT =
        1.0 / ((double) (AZMTH_ENC_COUNTS_PER_MODULE_REV));

    public static final TrapezoidProfile.Constraints X_DEFAULT_CONSTRAINTS =
        new TrapezoidProfile.Constraints(2, 2);
    public static final TrapezoidProfile.Constraints Y_DEFAULT_CONSTRAINTS =
        new TrapezoidProfile.Constraints(2, 2);

    public static final TrapezoidProfile.Constraints NO_CONSTRAINTS =
        new TrapezoidProfile.Constraints(Integer.MAX_VALUE, Integer.MAX_VALUE);
    public static final TrapezoidProfile.Constraints THETA_DEFAULT_CONSTRAINTS =
        new TrapezoidProfile.Constraints(4 * Math.PI, 16 * Math.PI);
  }

  public class Poses {
    public static final Translation2d SPEAKER = new Translation2d(0.334, 5.547);
  }
}
