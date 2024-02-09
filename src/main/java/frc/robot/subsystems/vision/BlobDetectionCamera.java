package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import monologue.Logged;
import monologue.Annotations.Log;

public class BlobDetectionCamera implements Logged {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private FieldObject2d simNotes;
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
        if (result.getBestTarget().getPitch() > 9) return false;
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
            Constants.cameraPitch, Units.degreesToRadians(target.getPitch())) - Units.inchesToMeters(7);
    }

    public Translation2d getNoteTargetOffset(PhotonTrackedTarget target) {
        return new Translation2d(getDistance(target), Rotation2d.fromDegrees(-target.getYaw()))
      .plus(new Translation2d(BlobDetectionCamera.Constants.cameraX, 0));
    }

    public List<Pose2d> getTargets(Pose2d robotPose) {
        if (RobotBase.isSimulation()) {
            return simNotes.getPoses();
        } else {
        if (!hasTarget()) return List.of();
        var list = result.getTargets();
        list.removeIf((t)-> (t.getPitch() > 9));
        return list.stream().map((t)->robotPose.transformBy(new Transform2d(getNoteTargetOffset(t), Rotation2d.fromDegrees(-t.getYaw())))).toList();
        }

        
    }

    public Optional<Pose2d> getBestTarget(Pose2d robotPose) {
        var targets = getTargets(robotPose);
        if (targets.size() == 0) {return Optional.empty();}
        return Optional.of(getTargets(robotPose).get(0));
    }
    
    public void update() {
        result = camera.getLatestResult();
    }
    public class Constants {
        public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera";
        public static final double cameraHeight = Units.inchesToMeters(17.875);
        public static final double cameraX = Units.inchesToMeters(0);
        public static final double cameraPitch = Units.degreesToRadians(-15);
        public static final double targetHeight = Units.inchesToMeters(2);
    }
}
