package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.*;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.AprilTags;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.trajectory.PPChasePoseCommand;
import monologue.Logged;

import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Subsystem that controls the drivetrain of the robot Handles all the odometry and base movement
 * for the chassis
 */
public class DrivebaseS extends SubsystemBase implements Logged {
  /** The abstract class for interfacing with the gyro and modules. */
  private final SwerveDriveIO io;

  /** The X controller used for autonomous movement. */
  public final PIDController m_xController = new PIDController(10, 0, 0.0);

  public final PIDController m_yController = new PIDController(10, 0, 0.0);
  public final PIDController m_thetaController = new PIDController(7, 0, 0);
  // constraints determined from OperatorControlC slew settings.
  // TODO replace this with a TrapezoidProfile delegating to m_thetaController?
  public final ProfiledPIDController m_profiledThetaController =
      new ProfiledPIDController(4, 0, 0, new Constraints(4, 8));

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          ModuleConstants.FL.centerOffset,
          ModuleConstants.FR.centerOffset,
          ModuleConstants.BL.centerOffset,
          ModuleConstants.BR.centerOffset);

  public final Vision m_vision;
  private final BiConsumer<String, List<Pose2d>> drawTrajectory;
  
  public SysIdRoutine m_linearIdRoutine;
  public SysIdRoutine m_angularIdRoutine;

  public DrivebaseS(
      Consumer<Runnable> addPeriodic, BiConsumer<String, List<Pose2d>> drawTrajectory) {
    io = Robot.isReal() ? new RealSwerveDriveIO(addPeriodic) : new SimSwerveDriveIO(addPeriodic);
    this.drawTrajectory = drawTrajectory;
    m_vision = new Vision(m_kinematics, io::getGyroHeading, io::getCurrentPositions);
    m_vision.resetPose(new Pose2d(4, 4, new Rotation2d()));
    m_thetaController.setTolerance(Units.degreesToRadians(0.5));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_profiledThetaController.setTolerance(Units.degreesToRadians(0.5));
    m_profiledThetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_xController.setTolerance(0.01);
    m_yController.setTolerance(0.01);
    m_linearIdRoutine =  new SysIdRoutine(
      new Config(
       Volts.per(Second).of(1),Volts.of(4),Second.of(10)),
      new Mechanism(volts->{
        double voltage = volts.baseUnitMagnitude();
        io.sysidLinear(voltage);
      },
      io::logDriveMotors,
      this,
      "drivebaseLin"
      )
    );
      m_angularIdRoutine =  new SysIdRoutine(
      new Config(),
      new Mechanism(volts->{
        double voltage = volts.baseUnitMagnitude();
        io.sysidAngular(voltage);
      },
      io::logDriveMotors,
      this,
      "drivebaseAng"
      )
    );
    resetPose(new Pose2d());
  }

  public Rotation3d getRotation3d() {
    return io.getRotation3d();
  }

  public double getGyroHeading() {
    return io.getGyroHeading().getRadians();
  }

  public double getPitch() {
    return io.getPitch();
  }

  @Override
  public void periodic() {
    m_vision.periodic();
    m_vision.updateCameraPoses(getPose());
    io.beforeCommands();
  }

  public void afterCommandsPeriodic(){
    io.afterCommands();
  }

  /**
   * Drive with the specified robot-relative ChassisSpeeds. All more complicated drive commands
   * should eventually call this.
   *
   * @param speeds
   */
  public void drive(ChassisSpeeds speeds) {
    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states;
    speeds = ChassisSpeeds.discretize(speeds, 0.12);
    if (Math.abs(speeds.vxMetersPerSecond) < 0.01
    && Math.abs(speeds.vyMetersPerSecond) < 0.01
    && Math.abs(speeds.omegaRadiansPerSecond) < 0.01) {
      states = getStoppedStates();
    } else {
      states = m_kinematics.toSwerveModuleStates(speeds);
    }
    // make sure the wheels don't try to spin faster than the maximum speed possible

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        speeds,
        MAX_MODULE_SPEED_MPS,
        MAX_FWD_REV_SPEED_MPS,
        MAX_ROTATE_SPEED_RAD_PER_SEC);
    setModuleStates(states);
  }

  /** Drive field-relative, with no mirroring for alliance. */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseHeading()));
  }

  /**
   * Drive field relative, with +x always facing away from the alliance wall.
   *
   * @param fieldRelativeSpeeds
   */
  public void driveAllianceRelative(ChassisSpeeds fieldRelativeSpeeds) {
    if (AllianceWrapper.getAlliance() == Alliance.Red) {
      drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              fieldRelativeSpeeds, getPoseHeading().plus(Rotation2d.fromRadians(Math.PI))));
    } else {
      drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseHeading()));
    }
  }

  public void driveFieldRelativeHeading(ChassisSpeeds speeds) {
    double omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    double currentTargetRadians = m_thetaController.getSetpoint();

    double newTargetRadians = currentTargetRadians + (omegaRadiansPerSecond / 50);

    double commandRadiansPerSecond =
        m_thetaController.calculate(getPoseHeading().getRadians(), newTargetRadians);

    speeds.omegaRadiansPerSecond = commandRadiansPerSecond;
    driveFieldRelative(speeds);
  }

  /**
   * Return the desired states of the modules when the robot is stopped. This can be an x-shape to
   * hold against defense, or all modules forward. Here we have it stopping all modules but leaving
   * the angles at their current positions.
   *
   * @return
   */
  private SwerveModuleState[] getStoppedStates() {
    SwerveModuleState[] states = getModuleStates().clone();
    for (int i = 0; i < NUM_MODULES; i++) {
      states[i].speedMetersPerSecond = 0;
    }
    return states;
  }

  /**
   * Method to set the desired state for each swerve module Uses PID and feedforward control to
   * control the linear and rotational values for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    io.setModuleStates(moduleStates, false);
  }

  /*
   * Returns an array of SwerveModuleStates.
   * Front(left, right), Rear(left, right)
   * This order is important to remain consistent across the codebase, due to conventions assumed in WPILib
   */
  public SwerveModuleState[] getModuleStates() {
    return io.getModuleStates();
  }

  /**
   * Return the module positions for odometry.
   *
   * @return an array of 4 SwerveModulePosition objects
   */
  public SwerveModulePosition[] getModulePositions() {
    return io.getCurrentPositions();
  }

  /**
   * Return the current position of the robot on field Based on drive encoder, gyro reading and
   * vision processing.
   */
  public Pose2d getPose() {
    return m_vision.getPose();
  }

  public Optional<Pose2d> getOldPose(double timestamp) {
    return m_vision.getOldPose(timestamp);
  }

  /**
   * Return the simulated estimate of the robot's pose. NOTE: on a real robot this will return a new
   * Pose2d, (0, 0, 0)
   *
   * @return
   */
  public Pose2d getSimPose() {
    return io.getSimPose();
  }

  /**
   * Reset odometry to the starting point of the given trajectory, as mirrored according to the
   * alliance.
   *
   * @param trajectory
   * @return
   */
  public Command resetPoseToBeginningC(PathPlannerTrajectory trajectory) {
    return Commands.runOnce(
        () ->
            resetPose(
                NomadMathUtil.mirrorPose(
                    new Pose2d(
                        trajectory.getInitialState().positionMeters,
                        trajectory.getInitialState().targetHolonomicRotation),
                    AllianceWrapper.getAlliance())));
  }

  /**
   * Reset the pose of odometry and sim to the given pose.
   *
   * @param pose The Pose2d to reset to.
   */
  public void resetPose(Pose2d pose) {
    io.resetPose(pose);

    m_vision.resetPose(pose);
  }

  /** Reset the measured distance driven for each module. */
  public void resetDriveDistances() {
    io.resetDistances();
  }

  /**
   * @return the current navX heading (which will not match odometry after drift or reset)
   */
  public Rotation2d getHeading() {
    return io.getGyroHeading();
  }

  /**
   * @return the current navX heading (which will not match odometry after drift or reset)
   */
  
  public double getHeadingDouble() {
    return getHeading().getRadians();
  }

  /**
   * Gets the current heading based on odometry. (this value will reflect odometry resets)
   *
   * @return the current odometry heading.
   */
  public Rotation2d getPoseHeading() {
    return getPose().getRotation();
  }

  /*
   * Resets the navX to 0 position;
   */
  public void resetImu() {
    io.resetIMU();
  }

  /** Returns a Translation2d representing the linear robot speed in field coordinates. */
  public ChassisSpeeds getFieldRelativeLinearSpeedsMPS() {
    // Get robot relative speeds from module states
    ChassisSpeeds robotRelativeSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
    // Get field relative speeds by undoing the field-robot conversion (which was
    // just a rotation by the heading)
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            getPoseHeading().unaryMinus());
    return fieldRelativeSpeeds;
  }

  /**
   * A convenience method to draw the robot pose and 4 poses representing the wheels onto the
   * field2d.
   *
   * @param field
   */
  public void drawRobotOnField(Field2d field) {
    field.setRobotPose(getPose());
    // Draw a pose that is based on the robot pose, but shifted by the translation
    // of the module relative to robot center,
    // then rotated around its own center by the angle of the module.
    // Name starts with z so it draws on top on the field display
    field
        .getObject("zmodules")
        .setPoses(
            List.of(
                getPose()
                    .transformBy(
                        new Transform2d(
                            ModuleConstants.FL.centerOffset, getModuleStates()[FL].angle)),
                getPose()
                    .transformBy(
                        new Transform2d(
                            ModuleConstants.FR.centerOffset, getModuleStates()[FR].angle)),
                getPose()
                    .transformBy(
                        new Transform2d(
                            ModuleConstants.BL.centerOffset, getModuleStates()[BL].angle)),
                getPose()
                    .transformBy(
                        new Transform2d(
                            ModuleConstants.BR.centerOffset, getModuleStates()[BR].angle))));
  }

  /** Reset each module's relative encoder and steering controller against the absolute encoder. */
  public void resetRelativeRotationEncoders() {
    io.reinitRotationEncoders();
    io.resetModuleSteerControllers();
  }

  /** Reset error buildup on the three */
  public void resetPID() {
    m_xController.reset();
    m_yController.reset();
    m_thetaController.reset();
  }

  public Pose2d getTargetPose() {
    return new Pose2d(
        m_xController.getSetpoint(),
        m_yController.getSetpoint(),
        new Rotation2d(m_thetaController.getSetpoint()));
  }

  /**** COMMANDS */

  public Command stopOnceC() {
    return runOnce(() -> this.drive(new ChassisSpeeds()));
  }

  public Command stopC() {
    return run(() -> this.drive(new ChassisSpeeds()));
  }

  public Command xLockC() {
    return run(
        () ->
            this.setModuleStates(
              // TODO unnecessary allocation
                new SwerveModuleState[] {
                  new SwerveModuleState(0, new Rotation2d(Math.PI / 2)), // FL
                  new SwerveModuleState(0, new Rotation2d(-Math.PI / 2)),
                  new SwerveModuleState(0, new Rotation2d(-Math.PI / 2)), // BL
                  new SwerveModuleState(0, new Rotation2d(Math.PI / 2))
                }));
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Command factory to drive a PathPlanner path. Paths are assumed to be created on the blue side,
   * and will be automatically flipped.
   *
   * @param path the path to run.
   * @return
   */
  public Command pathPlannerCommand(PathPlannerPath path) {
    FollowPathHolonomic command =
        new FollowPathHolonomic(
            path,
            this::getPose,
            this::getRobotRelativeChassisSpeeds,
            this::drive,
            Pathing.m_pathPlannerConfig,
            AllianceWrapper::isRed,
            this);
    return command;
  }

  public Command choreoCommand(String choreoTrajectory) {
    return pathPlannerCommand(PathPlannerPath.fromChoreoTrajectory(choreoTrajectory));
  }


  /**
   * Creates a new pose-chase command. This command generates and follows the target pose supplied
   * by targetSupplier. If the target has moved since the last generation, regen the trajectory. If
   * the trajectory is finished, switch to direct x-y-theta PID to hold the pose.
   *
   * @param targetSupplier the Supplier for the target Pose2d.
   * @return the PPChasePoseCommand
   */
  public Command chasePoseC(Supplier<Pose2d> targetSupplier) {
    return new PPChasePoseCommand(
            targetSupplier,
            this::getPose,
            Pathing.m_holonomicDriveController,
            m_xController,
            m_yController,
            m_thetaController,
            this::drive,
            (PathPlannerTrajectory traj) -> {
              drawTrajectory.accept("align", Pathing.ppTrajectoryToPoseList(traj));
            },
            (startPose, endPose) ->
                Pathing.generateTrajectoryToPose(
                    startPose,
                    endPose,
                    getRobotRelativeChassisSpeeds(),
                    new PathConstraints(2, 2, 2 * Math.PI, 2 * Math.PI)),
            this)
        .deadlineWith(LightStripS.getInstance().stateC(() -> States.Climbing));
  }

  /**
   * Command factory for manual drive.
   *
   * @param fwdXAxis the InputAxis for downfield movement (+1 is away from driver POV)
   * @param fwdYAxis the InputAxis for cross-field movement (+1 is left from driver POV)
   * @param rotAxis the InputAxis for rotation (+1 is full spin CCW)
   * @return A command for manual drive.
   */
  public Command manualDriveC(InputAxis fwdXAxis, InputAxis fwdYAxis, InputAxis rotAxis) {
    return runOnce(
            () -> {
              fwdXAxis.resetSlewRate();
              fwdYAxis.resetSlewRate();
              rotAxis.resetSlewRate();
            })
        .andThen(
            run(
                () -> {
                  /**
                   * Units are given in meters per second and radians per second Since joysticks
                   * give output from -1 to 1, we multiply the outputs by the max speed Otherwise,
                   * our max speed would be 1 meter per second and 1 radian per second
                   */
                  double fwdX = fwdXAxis.getAsDouble();
                  double fwdY = fwdYAxis.getAsDouble();

                  // scale the desired translation vector by max linear speed.
                  double driveDirectionRadians = Math.atan2(fwdY, fwdX);
                  double driveMagnitude = Math.hypot(fwdX, fwdY) * MAX_LINEAR_SPEED;
                  fwdX = driveMagnitude * Math.cos(driveDirectionRadians);
                  fwdY = driveMagnitude * Math.sin(driveDirectionRadians);

                  double rot;
                  rot = rotAxis.getAsDouble();
                  rot *= MAX_TURN_SPEED;
                  driveAllianceRelative(new ChassisSpeeds(fwdX, fwdY, rot));
                }));
  }


    /**
   * Command factory for manual drive with PID heading lock.
   *
   * @param fwdXAxis the InputAxis for downfield movement (+1 is away from driver POV)
   * @param fwdYAxis the InputAxis for downfield movement (+1 is left from driver POV)
   * @param headingAllianceRelative the heading to hold, relative to the alliance wall (0 faces away
   *     from driver station)
   * @return A command for manual drive with heading lock.
   */
  public Command manualHeadingDriveC(
      InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleSupplier headingAllianceRelative) {
        return manualHeadingDriveC(fwdXAxis, fwdYAxis, headingAllianceRelative, ()->0);
      }
  /**
   * Command factory for manual drive with PID heading lock.
   *
   * @param fwdXAxis the InputAxis for downfield movement (+1 is away from driver POV)
   * @param fwdYAxis the InputAxis for downfield movement (+1 is left from driver POV)
   * @param headingAllianceRelative the heading to hold, relative to the alliance wall (0 faces away
   *     from driver station)
   * @return A command for manual drive with heading lock.
   */
  public Command manualHeadingDriveC(
      InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleSupplier headingAllianceRelative, DoubleSupplier headingFF) {
        return manualFieldHeadingDriveC(fwdXAxis, fwdYAxis, ()->headingAllianceRelative.getAsDouble() + 
                      ((AllianceWrapper.getAlliance() == Alliance.Red) ? Math.PI : 0.0), headingFF);
      }
    public Command manualFieldHeadingDriveC(
      InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleSupplier headingFieldRelative, DoubleSupplier headingFF) {
    return runOnce(
            () -> {
              m_profiledThetaController.reset(getPoseHeading().getRadians(), getFieldRelativeLinearSpeedsMPS().omegaRadiansPerSecond);
            })
        .andThen(
            run(
                () -> {
                  /**
                   * Units are given in meters per second radians per second Since joysticks give
                   * output from -1 to 1, we multiply the outputs by the max speed Otherwise, our
                   * max speed would be 1 meter per second and 1 radian per second
                   */
                  double fwdX = fwdXAxis.getAsDouble();
                  double fwdY = fwdYAxis.getAsDouble();
                  double driveDirectionRadians = Math.atan2(fwdY, fwdX);
                  double driveMagnitude = Math.hypot(fwdX, fwdY) * MAX_LINEAR_SPEED;
                  fwdX = driveMagnitude * Math.cos(driveDirectionRadians);
                  fwdY = driveMagnitude * Math.sin(driveDirectionRadians);

                  double rot;

                  rot =
                      m_profiledThetaController.calculate(
                          getPoseHeading().getRadians(),
                          headingFieldRelative.getAsDouble());
                  log("thetaGoal", m_profiledThetaController.getGoal().position);
                  log("thetaSetpt", m_profiledThetaController.getSetpoint().position);
                  log("thetaReal", getPoseHeading().getRadians());
                  rot += headingFF.getAsDouble();
                  driveAllianceRelative(new ChassisSpeeds(fwdX, fwdY, rot));
                }));
  }

  // public Command leftPlatformAlign() {
  //     return chasePickupC(()-> POIManager.ownPOI(AllianceWrapper.isRed() ? POIS.GRID_PLAT :
  // POIS.WALL_PLAT));
  // }
  // public Command rightPlatformAlign() {
  //     return chasePickupC(()-> POIManager.ownPOI(AllianceWrapper.isRed() ? POIS.WALL_PLAT :
  // POIS.GRID_PLAT));
  // }
}
