package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import monologue.Logged;
import monologue.Annotations.Log;

public class BlobDetectionCamera implements Logged {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    public BlobDetectionCamera(Consumer<Runnable> addPeriodic) {
        camera = new PhotonCamera(Constants.CAMERA_NAME);
        addPeriodic.accept(this::update);
    }

    /**
     * The yaw of the best target, CCW (left half of frame) positive
     * @return
     */
    @Log.NT
    public double getYaw() {
        if (!hasTarget()) return 0;
        return -Units.degreesToRadians(result.getBestTarget().getYaw());
    }

    /**
     * The pitch of the best target, (bottom half of frame positive)
     * @return
     */
    @Log.NT
    public double getPitch() {
        if (!hasTarget()) return 0;
        return -Units.degreesToRadians(result.getBestTarget().getPitch());
    }
    @Log.NT
    public boolean hasTarget() {
        if (result == null) return false;
        if (!result.hasTargets()) return false;
        if (result.getBestTarget().getPitch() > 3) return false;
        return true;
    }
    @Log.NT
    public double getDistance() {
        if (!hasTarget()) return 0;
        return getDistance(result.getBestTarget());
    }

    public double getDistance(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.cameraHeight,
            Constants.targetHeight,
            0, Units.degreesToRadians(target.getPitch())) - Units.inchesToMeters(6);
    }

    public Translation2d getNoteTargetOffset(PhotonTrackedTarget target) {
        return new Translation2d(getDistance(target), Rotation2d.fromDegrees(-target.getYaw()))
      .plus(new Translation2d(BlobDetectionCamera.Constants.cameraX, 0));
    }

    public List<Pose2d> getTargets(Pose2d robotPose) {
        if (!hasTarget()) return List.of();
        var list = result.getTargets();
        list.removeIf((t)-> (t.getPitch() > 3));
        return list.stream().map((t)->robotPose.transformBy(new Transform2d(getNoteTargetOffset(t), new Rotation2d()))).toList();
        
    }
    
    public void update() {
        result = camera.getLatestResult();
    }
    public class Constants {
        public static final String CAMERA_NAME = "Camera_Module_v1";
        public static final double cameraHeight = Units.inchesToMeters(7);
        public static final double cameraX = Units.inchesToMeters(13);
        public static final double targetHeight = Units.inchesToMeters(2);
    }
}
