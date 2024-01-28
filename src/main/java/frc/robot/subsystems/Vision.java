package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.AprilTags;
import monologue.Logged;

public class Vision implements Logged {
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Supplier<Rotation2d> getHeading;
    private Supplier<SwerveModulePosition[]> getModulePositions;
    private List<PhotonPoseEstimator> m_cameras;

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
            m_cameras.add(
                    new PhotonPoseEstimator(
                            Constants.layout,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera(entry.getKey()),
                            entry.getValue()));
        });
    }

    public void periodic() {
        m_poseEstimator.update(getHeading.get(), getModulePositions.get());
        if (RobotBase.isReal()) {
            for (PhotonPoseEstimator estimator : m_cameras) {
                var robotPoseOpt = estimator.update();
                if (robotPoseOpt.isEmpty()) {
                    continue;
                }
                var robotPose = robotPoseOpt.get();
                // var confidence = AprilTags.calculateVisionUncertainty(
                //         robotPose.estimatedPose.getX(),
                //         getPose().getRotation(),
                //         new Rotation2d(estimator.getRobotToCameraTransform().getRotation().getZ()));
                log("visionPose3d", robotPose.estimatedPose);
                m_poseEstimator.addVisionMeasurement(
                        robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds, VecBuilder.fill(0.1, 0.1, 0.1));
            }
        }
    }

    public void resetPose(Pose2d newPose) {
        m_poseEstimator.resetPosition(getHeading.get(), getModulePositions.get(), newPose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public class Constants {
        public static final Map<String, Transform3d> cameras = Map.of(
                "OV9281-3", new Transform3d(
                    Units.inchesToMeters(0.5), -Units.inchesToMeters(0.5), Units.inchesToMeters(22.75),
                    new Rotation3d(0, -Units.degreesToRadians(15), Math.PI)
                ));
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
