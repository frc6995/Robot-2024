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
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.AprilTags;
import monologue.Logged;
import monologue.Annotations.Log;
import java.util.function.Consumer;

public class CTREVision implements Logged {
    public record VisionMeasurement (Pose2d pose, double timestamp, Vector<N3> stddevs){};

    private List<Pair<String, PhotonPoseEstimator>> m_cameras;
    private List<PhotonCamera> m_actualCameras;
    private double redSpeakerDist;
    private double blueSpeakerDist;
    private double redSpeakerDistTime;
    private double blueSpeakerDistTime;
    private Consumer<VisionMeasurement> addVisionMeasurement;
    private Supplier<Pose2d> getPose;
    public CTREVision(
            Consumer<VisionMeasurement> addVisionMeasurement, Supplier<Pose2d> getPose) {
        this.addVisionMeasurement = addVisionMeasurement;
        this.getPose = getPose;
        m_cameras = new ArrayList<>();
        m_actualCameras = new ArrayList<>();
        Constants.cameras.entrySet().iterator().forEachRemaining((entry) -> {
            var cam = new PhotonCamera(entry.getKey());
            var estimator = 
            new PhotonPoseEstimator(
                            Constants.layout,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam,
                            entry.getValue());
                            estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            m_actualCameras.add(cam);
            m_cameras.add(new Pair<String, PhotonPoseEstimator>(entry.getKey(), estimator));
        });
    }

    public boolean seesRedSpeaker() {
        return Timer.getFPGATimestamp() - redSpeakerDistTime < 0.5;
    }
    public boolean seesBlueSpeaker() {
        return Timer.getFPGATimestamp() - blueSpeakerDistTime < 0.5;
    }
    public boolean seesOwnSpeaker() {
        return AllianceWrapper.isRed() ? seesRedSpeaker() : seesBlueSpeaker();
    }

    public double ownSpeakerDistance() {
        return AllianceWrapper.isRed() ? redSpeakerDist : blueSpeakerDist;
    }
    public boolean hasTarget() {
        for (var cam : m_actualCameras) {
            if (cam.getLatestResult().hasTargets()) {
                return true;
            }
        }
        return false;
    }

    public void periodic() {
        if (RobotBase.isReal()) {
            for (Pair<String, PhotonPoseEstimator> pair : m_cameras) {
                var estimator = pair.getSecond();
                estimator.setReferencePose(getPose.get());
                var robotPoseOpt = estimator.update();
                if (robotPoseOpt.isEmpty()) {
                    continue;
                }

                
                var robotPose = robotPoseOpt.get();
                for (var target : robotPose.targetsUsed) {
                    if (target.getFiducialId() == 7 || target.getFiducialId() == 4) {
                        
                        var transform = estimator.getRobotToCameraTransform();
                        double tgtPitch = 0;
                        double camPitch = 0;
                        // 90* roll cam
                        if (transform.getRotation().getX() < -1) {
                            tgtPitch = Units.degreesToRadians(target.getYaw());
                            camPitch = Units.degreesToRadians(-35);
                        } else {
                            tgtPitch = Units.degreesToRadians(target.getPitch());
                            camPitch = transform.getRotation().getY();
                            log(pair.getFirst() + "camPitch", camPitch);
                        }
                        var distance = PhotonUtils.calculateDistanceToTargetMeters(
                            estimator.getRobotToCameraTransform().getZ(),
                            Constants.layout.getTagPose(7).get().getZ(),
                            -camPitch, tgtPitch);

                        if (target.getFiducialId() ==7 ){
                            log(pair.getFirst() + "blueSpkrDist", distance);
                            blueSpeakerDist = distance;
                            blueSpeakerDistTime = Timer.getFPGATimestamp();
                        } else if (target.getFiducialId() ==4 ) {
                            log(pair.getFirst() + "redSpkrDist", distance);
                            redSpeakerDist = distance;
                            redSpeakerDistTime = Timer.getFPGATimestamp();
                        }
                    }
                }
                if (Math.abs(robotPose.estimatedPose.getZ()) > 0.5) {
                    continue;
                }
                double xConfidence;
                double yConfidence;
                double angleConfidence;
                if(robotPose.targetsUsed.size() == 0) {
                    continue; //should never happen but measurement shouldn't be trusted
                }
                double closestDistance = 1000;
                double avgDistance = 0;
                double closeEnoughTgts = 0;
                boolean ignore = false;
                for (var tgt : robotPose.targetsUsed ) {
                    double tdist = tgt.getBestCameraToTarget().getTranslation().getNorm();
                    avgDistance += tdist;
                    if (tdist < closestDistance) {
                        closestDistance = tdist;
                    }
                    if (tdist <= Units.feetToMeters(15)) {
                        closeEnoughTgts++;
                    }
                    ignore |= (tgt.getFiducialId() == 13);
                    ignore |= (tgt.getFiducialId() == 14);
                }
                if (ignore) {continue;}
                double distance = avgDistance / robotPose.targetsUsed.size();
                log("avgDist-"+pair.getFirst(), distance);
                if (closeEnoughTgts ==0) {
                    continue;
                }
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
                log("timestamp"+pair.getFirst(), robotPose.timestampSeconds);
                addVisionMeasurement.accept( new VisionMeasurement(
                        robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds,
                        VecBuilder.fill(xConfidence, yConfidence, angleConfidence)));
            }
        }
    }
    public void captureImages() {
        for (var cam : m_actualCameras) {
            cam.takeOutputSnapshot();
        }
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
            )
            // "OV9281-BR", new Transform3d(
            //     Units.inchesToMeters(-12.5+9.2),
            //     Units.inchesToMeters(-2),
            //     Units.inchesToMeters(23.4),
            //     new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(180+31))
            // ),
            // "OV9281-FR", new Transform3d(
            //     Units.inchesToMeters(12.5-13.875),
            //     Units.inchesToMeters(-2),
            //     Units.inchesToMeters(23.4),
            //     new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(-31))
            // ),
            // "OV9281-FL", new Transform3d(
            //     Units.inchesToMeters(12.5-13.875),
            //     Units.inchesToMeters(2),
            //     Units.inchesToMeters(23.4),
            //     new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(31))
            // ),
            // "OV9281-BL", new Transform3d(
            //     Units.inchesToMeters(-12.5+9.2+2.375),
            //     Units.inchesToMeters(2),
            //     Units.inchesToMeters(23.4),
            //     new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(180-31))
            // )
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
