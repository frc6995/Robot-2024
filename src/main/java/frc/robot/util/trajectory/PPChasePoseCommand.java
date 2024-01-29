package frc.robot.util.trajectory;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DrivebaseS;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link PathPlannerTrajectory} with a
 * swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class PPChasePoseCommand extends Command {
  private final Timer m_timer = new Timer();
  private Supplier<Pose2d> m_targetPose;
  private PathPlannerTrajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final PPHolonomicDriveController m_controller;
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_thetaController;
  private final Consumer<ChassisSpeeds> m_outputChassisSpeedsRobotRelative;
  private final Consumer<PathPlannerTrajectory> m_outputTrajectory;
  private final BiFunction<Pose2d, Pose2d, PathPlannerTrajectory> m_trajectoryGenerator;
  private Pose2d m_lastRegenTarget;
  private DrivebaseS m_drive;
  private Trigger m_finishTrigger;

  /**
   * Constructs a command to follow a moving target pose. Uses PathPlanner trajectories when the
   * target is more than 0.2 m away.
   *
   * @param targetPose A Supplier for the target pose.
   * @param pose A Supplier for the current pose.
   * @param driveController The PPHolonomicDriveController to use.
   * @param outputChassisSpeedsFieldRelative
   * @param trajectoryDebugOutput
   * @param trajectoryGenerator
   * @param drive
   */
  public PPChasePoseCommand(
      Supplier<Pose2d> targetPose,
      Supplier<Pose2d> pose,
      PPHolonomicDriveController driveController,
      PIDController xController,
      PIDController yController,
      PIDController thetaController,
      Consumer<ChassisSpeeds> outputChassisSpeedsFieldRelative,
      Consumer<PathPlannerTrajectory> trajectoryDebugOutput,
      BiFunction<Pose2d, Pose2d, PathPlannerTrajectory> trajectoryGenerator,
      DrivebaseS drive) {

    m_targetPose = targetPose;
    m_pose = pose;
    m_lastRegenTarget = m_targetPose.get();

    m_controller = driveController;
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_outputTrajectory = trajectoryDebugOutput;
    m_trajectoryGenerator = trajectoryGenerator;
    m_outputChassisSpeedsRobotRelative = outputChassisSpeedsFieldRelative;
    m_drive = drive;
    m_finishTrigger = new Trigger(() -> false);
    // new
    // Trigger(()->m_pose.get().getTranslation().getDistance(m_targetPose.get().getTranslation())
    // < Units.inchesToMeters(1)
    // && Math.abs(m_pose.get().getRotation().getDegrees() -
    // m_targetPose.get().getRotation().getDegrees()) < 1)
    // .debounce(0.05);
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    regenTrajectory();
  }

  private void regenTrajectory() {
    m_timer.reset();
    m_timer.start();
    m_lastRegenTarget = m_targetPose.get();
    m_trajectory = m_trajectoryGenerator.apply(m_pose.get(), m_targetPose.get());
    m_outputTrajectory.accept(m_trajectory);
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    // If the target's moved more than 0.2 m since the last regen, generate the
    // trajectory again.
    if (m_targetPose.get().getTranslation().getDistance(m_lastRegenTarget.getTranslation()) > 0.2) {
      regenTrajectory();
    }

    State desiredState;
    ChassisSpeeds targetChassisSpeeds;
    // Make sure the trajectory is not empty
    // Make sure it's still time to be following the trajectory.
    if (m_trajectory.getStates().size() != 0
        && m_timer.get() < m_trajectory.getTotalTimeSeconds()) {
      double curTime = m_timer.get();
      desiredState = (State) m_trajectory.sample(curTime);
      targetChassisSpeeds = m_controller.calculateRobotRelativeSpeeds(m_pose.get(), desiredState);
    }
    // if the trajectory is empty, or the time is up, just use the holonomic drive
    // controller to hold the pose.
    else {
      var pose = m_pose.get();
      var target = m_targetPose.get();
      targetChassisSpeeds =
          new ChassisSpeeds(
              m_xController.calculate(pose.getX(), target.getX()),
              m_yController.calculate(pose.getY(), target.getY()),
              m_thetaController.calculate(
                  pose.getRotation().getRadians(), target.getRotation().getRadians()));
      if (pose.getTranslation().getDistance(target.getTranslation()) < Units.inchesToMeters(0.5)) {
        targetChassisSpeeds.vxMetersPerSecond = 0;
        targetChassisSpeeds.vyMetersPerSecond = 0;
      }
    }
    m_outputChassisSpeedsRobotRelative.accept(targetChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_outputChassisSpeedsRobotRelative.accept(new ChassisSpeeds());
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_finishTrigger.getAsBoolean();
  }
}
