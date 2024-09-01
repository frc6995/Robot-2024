package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Annotations.Log;

public class BlobDetectionCamera implements Logged {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private FieldObject2d simNotes;
    public final Trigger hasTarget;
    private double[][] threeTargetAngles = new double[3][2];
    public BlobDetectionCamera(Consumer<Runnable> addPeriodic, FieldObject2d simNotes) {
        camera = new PhotonCamera(Constants.CAMERA_NAME);
        addPeriodic.accept(this::update);
        this.simNotes = simNotes;
        if (RobotBase.isSimulation()) {
            simNotes.setPoses(
                new Pose2d(1, 1, new Rotation2d()),
                new Pose2d(1, 2, new Rotation2d()),
                new Pose2d(1, 3, new Rotation2d()));
        }
        hasTarget = new Trigger(this::hasTarget);
    }

    /**
     * The yaw of the best target, CCW (left half of frame) positive
     * @return
     */
    public double getYaw() {
        if (!hasTarget()) return 0;
        return -Units.degreesToRadians(result.getBestTarget().getYaw());
    }

    /**
     * The pitch of the best target, (bottom half of frame positive)
     * @return
     */
    public double getPitch() {
        if (!hasTarget()) return 0;
        return -Units.degreesToRadians(result.getBestTarget().getPitch());
    }
    public boolean hasTarget() {
        if (result == null) return false;
        if (!result.hasTargets()) return false;
        if (result.getBestTarget().getPitch() > 9) return false;
        return true;
    }
    @Log
    public double getDistance() {
        if (!hasTarget()) return 0;
        return getDistance(result.getBestTarget());
    }

    @Log
    public double getDistanceInches() {
        
        return Units.metersToInches(getDistance());
    }

    public double[][] getThreeTargetAngles() {
            threeTargetAngles[0][0] = -1000;
            threeTargetAngles[1][0] = -1000;
            threeTargetAngles[2][0] = -1000;
        if (!hasTarget()) {

            return threeTargetAngles;
        }
        for (int i = 0; i < Math.min(3, result.targets.size()); i++) {
            threeTargetAngles[i][0] = result.targets.get(i).getYaw();
            threeTargetAngles[i][1] = result.targets.get(i).getArea() * 5;
        }
        return threeTargetAngles;

    }
    public double getDistance(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.cameraHeight,
            Constants.targetHeight,
            Constants.cameraPitch, Units.degreesToRadians(target.getPitch()))/Math.cos(Units.degreesToRadians(target.getYaw()))
        + Units.inchesToMeters(7);
    }

    public Translation2d getNoteTargetOffset(PhotonTrackedTarget target) {
        return new Translation2d(getDistance(target), Rotation2d.fromDegrees(-target.getYaw()))
      .plus(new Translation2d(BlobDetectionCamera.Constants.cameraX, 0));
    }

    public List<Pose2d> getTargets(Function<Double, Optional<Pose2d>> robotPose) {
        if (result == null) {
            return List.of();
        }
        var pose = robotPose.apply(result.getTimestampSeconds());
        if (pose.isEmpty()) {
            return List.of();
        }
        if (RobotBase.isSimulation()) {
            return simNotes.getPoses();
        } else {
        if (!hasTarget()) return List.of();
        var list = result.getTargets();
        list.removeIf((t)-> (t.getPitch() > 9));
        return list.stream().map((t)->pose.get().transformBy(new Transform2d(getNoteTargetOffset(t), Rotation2d.fromDegrees(-t.getYaw())))).toList();
        }

        
    }

    public Optional<Pose2d> getBestTarget(Pose2d robotPose) {
        var targets = getTargets((time)->Optional.of(robotPose));
        if (targets.size() == 0) {return Optional.empty();}
        return Optional.of(targets.get(0));
    }
    
    public void update() {
        result = camera.getLatestResult();
    }
    public class Constants {
        public static final String CAMERA_NAME = "OV9782";
        public static final double cameraHeight = Units.inchesToMeters(19.75);
        public static final double cameraX = Units.inchesToMeters(-0.5);
        public static final double cameraPitch = Units.degreesToRadians(-18.75);
        public static final double targetHeight = Units.inchesToMeters(0);
        public static final double fov_h = Units.degreesToRadians(76.4);
        public static final double fov_v = Units.degreesToRadians(57.3);
    }
}
