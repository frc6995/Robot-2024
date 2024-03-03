package frc.robot.subsystems.vision;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.vision.PoseEstimator;
import frc.robot.subsystems.vision.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.AprilTags;
import monologue.Logged;

public class Vision implements Logged {
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Supplier<Rotation2d> getHeading;
    private Supplier<SwerveModulePosition[]> getModulePositions;
    private List<Pair<String, PhotonPoseEstimator>> m_cameras;

    public Vision(
            SwerveDriveKinematics kinematics,
            Supplier<Rotation2d> getHeading,
            Supplier<SwerveModulePosition[]> getModulePositions) {
        this.getHeading = getHeading;
        this.getModulePositions = getModulePositions;
        m_poseEstimator = new SwerveDrivePoseEstimator(kinematics,
                getHeading.get(),
                getModulePositions.get(),
                new Pose2d());
        m_cameras = new ArrayList<>();
        Constants.cameras.entrySet().iterator().forEachRemaining((entry) -> {
            var estimator = 
            new PhotonPoseEstimator(
                            Constants.layout,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera(entry.getKey()),
                            entry.getValue());
                            estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            m_cameras.add(new Pair<String, PhotonPoseEstimator>(entry.getKey(), estimator));
        });
    }

    public Optional<Pose2d> getOldPose(double timestamp) {
        if (Timer.getFPGATimestamp() - timestamp > 1.5) {
            return Optional.empty();
        }
        var interp = m_poseEstimator.m_poseBuffer.getSample(timestamp);
        if (interp.isEmpty()) {return Optional.empty();}
        return Optional.of(interp.get().poseMeters);
    }

    public void periodic() {
        m_poseEstimator.update(getHeading.get(), getModulePositions.get());
        if (RobotBase.isReal()) {
            for (Pair<String, PhotonPoseEstimator> pair : m_cameras) {
                var estimator = pair.getSecond();
                estimator.setReferencePose(getPose());
                var robotPoseOpt = estimator.update();
                if (robotPoseOpt.isEmpty()) {
                    continue;
                }
                
                var robotPose = robotPoseOpt.get();
                double xConfidence;
                double yConfidence;
                double angleConfidence;
                if(robotPose.targetsUsed.size() == 0) {
                    continue; //should never happen but measurement shouldn't be trusted
                }
                double distance = robotPose.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm();
                if (robotPose.targetsUsed.size() < 2) {
                    xConfidence = 0.5 * distance;
                    yConfidence = 0.5 * distance;
                    angleConfidence = 0.5 * distance;
                }
                else {
                    xConfidence = 0.1 * distance;
                    yConfidence = 0.1 * distance;
                    angleConfidence = 0.3*distance;
                }
                // var confidence = AprilTags.calculateVisionUncertainty(
                //         robotPose.estimatedPose.getX(),
                //         getPose().getRotation(),
                //         new Rotation2d(estimator.getRobotToCameraTransform().getRotation().getZ()));
                log("visionPose3d-"+pair.getFirst(), robotPose.estimatedPose);
    
                m_poseEstimator.addVisionMeasurement(
                        robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds,
                        VecBuilder.fill(xConfidence, yConfidence, angleConfidence));
            }
        }
    }

    public void resetPose(Pose2d newPose) {
        m_poseEstimator.resetPosition(getHeading.get(), getModulePositions.get(), newPose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
    public void updateCameraPoses(Pose2d drivebasePose) {
        for (Pair<String, PhotonPoseEstimator> pair : m_cameras) {
            log("cam-"+pair.getFirst(), 
                new Pose3d(drivebasePose).transformBy(pair.getSecond().getRobotToCameraTransform()));
        }
    }

    public class Constants {
        /**
         * 
         */
        public static final Map<String, Transform3d> cameras= Map.of(
            "Arducam_OV2311_USB_Camera", new Transform3d(
                Units.inchesToMeters(-12.5+9.1),
                Units.inchesToMeters(0),
                Units.inchesToMeters(23.3),
                new Rotation3d(Units.degreesToRadians(-90), Units.degreesToRadians(-35), Units.degreesToRadians(180))
            ),
            "OV9281-BR", new Transform3d(
                Units.inchesToMeters(-12.5+9.2),
                Units.inchesToMeters(-2),
                Units.inchesToMeters(23.4),
                new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(180+31))
            ),
            "OV9281-FR (1)", new Transform3d(
                Units.inchesToMeters(12.5-13.875),
                Units.inchesToMeters(-2),
                Units.inchesToMeters(23.4),
                new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(-31))
            ),
            "OV9281-FL", new Transform3d(
                Units.inchesToMeters(12.5-13.875),
                Units.inchesToMeters(2),
                Units.inchesToMeters(23.4),
                new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(31))
            ),
            "OV9281-BL", new Transform3d(
                Units.inchesToMeters(-12.5+9.2+2.375),
                Units.inchesToMeters(2),
                Units.inchesToMeters(23.4),
                new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(180-31))
            )
        );// = Map.of(
                // "OV9281-4", new Transform3d(
                //     Units.inchesToMeters(1.5),
                //     -Units.inchesToMeters(-0.5),
                //     Units.inchesToMeters(22.625),
                //     new Rotation3d(Units.degreesToRadians(1), -Units.degreesToRadians(15), Math.PI)
                // ));
        public static AprilTagFieldLayout layout;
        static {
            try {
                layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            } catch (IOException e) {
                DriverStation.reportError("AprilTagFieldLayout could not be loaded", false);
                layout = new AprilTagFieldLayout(List.of(), 16.54175, 8.21055);
            }
        }

    }
}
