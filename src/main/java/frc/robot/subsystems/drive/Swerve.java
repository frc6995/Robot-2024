package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.ctre.phoenix6.signals.NeutralModeValue;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.LightStripS.States;
import frc.robot.subsystems.vision.CTREVision;
import frc.robot.subsystems.vision.CTREVision.VisionMeasurement;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import monologue.Logged;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import java.util.List;

import java.util.function.BiConsumer;
import static frc.robot.generated.TunerConstants.kDriveRadiusMeters;
import static frc.robot.generated.TunerConstants.kDriveRotationsPerMeter;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.Defaults.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * Subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem, Logged {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	private final ApplyChassisSpeeds m_autoRequest = new ApplyChassisSpeeds()
		.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.RobotCentric m_characterisationReq = new SwerveRequest.RobotCentric()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveDriveBrake m_brake = new SwerveDriveBrake();
	private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric()
		.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.FieldCentric m_allianceRelative = new SwerveRequest.FieldCentric()
		.withDriveRequestType(DriveRequestType.Velocity);
	
	private Rotation2d m_desiredRot = ZERO_ROTATION2D;

	private final double m_characterisationSpeed = 1;
	public final DoubleSupplier m_gyroYawRadsSupplier;
    public final CTREVision m_vision;
    private BiConsumer<String, List<Pose2d>> drawTrajectory = (name, poses)->{};
	private final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(1);
	private final SwerveModuleConstants[] moduleConstants;
    @Log
	private double lastGyroYawRads = 0;
    @Log
	private double accumGyroYawRads = 0;

	private double[] startWheelPositions = new double[4];
	private double currentEffectiveWheelRadius = 0;

    public final PIDController m_xController = new PIDController(10, 0, 0.0);

    public final PIDController m_yController = new PIDController(10, 0, 0.0);
	public final PIDController m_thetaController = new PIDController(7, 0, 0);
    public final ProfiledPIDController m_profiledThetaController = new ProfiledPIDController(7, 0, 0, new TrapezoidProfile.Constraints(8, 12));
	private final SysIdSwerveTranslation characterization = new SysIdSwerveTranslation();
	private final SwerveModuleLog fr = new SwerveModuleLog("0-FR");
	private final SwerveModuleLog fl = new SwerveModuleLog("1-FL");
	private final SwerveModuleLog br = new SwerveModuleLog("2-BR");
	private final SwerveModuleLog bl = new SwerveModuleLog("3-BL");
	@IgnoreLogged
	private final SwerveModuleLog[] mods = new SwerveModuleLog[] {fr, fl, br, bl};
	// private final SysIdSwerveRotation characterization = new
	// SysIdSwerveRotation();
	// private final SysIdSwerveSteerGains characterization = new
	// SysIdSwerveSteerGains();
	private final SysIdRoutine m_sysId = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,
			Volts.of(7),
			null,
			(state) -> SignalLogger.writeString("state", state.toString())),
		new SysIdRoutine.Mechanism(
			(volts) -> setControl(characterization.withVolts(volts)),
			null,
			this));
	public void addVisionMeasurement(VisionMeasurement measurement) {
		addVisionMeasurement(
			measurement.pose(), measurement.timestamp(), measurement.stddevs());
	}

	public Swerve( SwerveDrivetrainConstants driveTrainConstants, double odometryFreq, SwerveModuleConstants... modules) {
		super(driveTrainConstants, odometryFreq, modules);
		this.moduleConstants = modules;
        m_vision = new CTREVision(this::addVisionMeasurement, this::getPose);
		if (Utils.isSimulation()) {
			startSimThread();
		}
		m_gyroYawRadsSupplier = () -> Units.degreesToRadians(getPigeon2().getAngle());
		m_thetaController.enableContinuousInput(0, 2*Math.PI);
		m_profiledThetaController.enableContinuousInput(0, 2 * Math.PI);
		m_fieldCentric.ForwardReference = ForwardReference.RedAlliance;
		m_allianceRelative.ForwardReference = ForwardReference.OperatorPerspective;
	}

	
	public void setTrajectoryDrawer(
		BiConsumer<String, List<Pose2d>> drawTrajectory) {
			this.drawTrajectory = drawTrajectory;
		}


    public Pose2d getPose () {
        return getState().Pose;
    }

	public void drive(ChassisSpeeds speeds) {
		setControl(m_autoRequest.withSpeeds(speeds));
	}

	public void driveAllianceRelative(ChassisSpeeds speeds) {
		setControl(m_allianceRelative
			.withVelocityX(speeds.vxMetersPerSecond)
			.withVelocityY(speeds.vyMetersPerSecond)
			.withRotationalRate(speeds.omegaRadiansPerSecond))
		;
	}
	public void driveFieldRelative(ChassisSpeeds speeds ) {
		setControl(m_fieldCentric
			.withVelocityX(speeds.vxMetersPerSecond)
			.withVelocityY(speeds.vyMetersPerSecond)
			.withRotationalRate(speeds.omegaRadiansPerSecond));
	}


	public Command wheelRadiusCharacterisation(double omegaDirection) {
		var initialize = runOnce(() -> {
			lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
			accumGyroYawRads = 0;
			currentEffectiveWheelRadius = 0;
			for (int i = 0; i < Modules.length; i++) {
				var pos = Modules[i].getPosition(true);
				startWheelPositions[i] = pos.distanceMeters * kDriveRotationsPerMeter;
			}
			m_omegaLimiter.reset(0);
		});

		var executeEnd = runEnd(
			() -> {
				setControl(m_characterisationReq
					.withRotationalRate(m_omegaLimiter.calculate(m_characterisationSpeed * omegaDirection)));
				accumGyroYawRads += MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
				lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
				double averageWheelPositionRotations = 0;
				double[] wheelPositionRotationss = new double[4];
				for (int i = 0; i < Modules.length; i++) {
					var pos = Modules[i].getPosition(true);
					log("pos"+i, pos.distanceMeters);
					wheelPositionRotationss[i] = pos.distanceMeters * kDriveRotationsPerMeter;
					averageWheelPositionRotations += Math.abs(wheelPositionRotationss[i] - startWheelPositions[i]);
				}
				averageWheelPositionRotations /= 4.0;
				log("wheelPos", wheelPositionRotationss);
				log("avgWheelPos", averageWheelPositionRotations);
				log("accumGyroYawRads", accumGyroYawRads);
				
				// arc length meters / wheel rotations = wheel circumference
				currentEffectiveWheelRadius = (accumGyroYawRads * kDriveRadiusMeters) / averageWheelPositionRotations;
				log("currWheelRad", currentEffectiveWheelRadius / (2*Math.PI));
				// log_lastGyro.accept(lastGyroYawRads);
				// log_avgWheelPos.accept(averageWheelPosition);
				// log_accumGyro.accept(accumGyroYawRads);
				// log_curEffWheelRad.accept(currentEffectiveWheelRadius);
			}, () -> {
				setControl(m_characterisationReq.withRotationalRate(0));
				if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
					System.out.println("not enough data for characterization " + accumGyroYawRads);
				} else {
					System.out.println(
						"effective wheel radius: "
							+ currentEffectiveWheelRadius
							+ " inches");
				}
			});

		return Commands.sequence(
			initialize, executeEnd);
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> setControl(requestSupplier.get()));
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			//SmartDashboard.putNumber("delta", deltaTime);
			m_lastSimTime = currentTime;
			
			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		Notifier.setHALThreadPriority(true, 99);
		m_simNotifier.startPeriodic(kSimLoopPeriod);

	}

	public void logModulePositions() {
		for (int i = 0; i < Modules.length; i++) {
			SmartDashboard.putNumber("Module " + i + "/position",
				getModule(i).getDriveMotor().getPosition().getValueAsDouble());
		}
	}

	public void setTestMode() {
		for (int i = 0; i < Modules.length; i++) {
			getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
			getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
		}
	}

	public Pose3d getPose3d() {
		var txr2d = getState().Pose.getTranslation();
		// we're on the floor. I hope. (i'm going to make the robot fly! >:D)
		return new Pose3d(txr2d.getX(), txr2d.getY(), 0, getRotation3d());
	}

	public void choreoController(Pose2d currentPose, SwerveSample sample) {
		var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
		  sample.vx + m_xController.calculate(currentPose.getX(), sample.x),
		  sample.vy + m_yController.calculate(currentPose.getY(), sample.y),
		  sample.omega + m_thetaController.calculate(currentPose.getRotation().getRadians(), sample.heading),
		  currentPose.getRotation()
		);
		drive(speeds);
	  }

  public Command resetPoseToBeginningC(Trajectory<SwerveSample> trajectory) {
    return Commands.runOnce(
        () ->
            seedFieldRelative(
               trajectory.getInitialPose(AllianceWrapper.isRed())));
  }

  public Command resetPoseToBeginningC(AutoTrajectory trajectory) {
    return Commands.runOnce(
        () -> {
			var initial = trajectory.getInitialPose();
			initial.ifPresent((pose)->
            seedFieldRelative(
               pose)
			);
		}
);
  }

  private TrapezoidProfile driveToPoseProfile = new TrapezoidProfile(new Constraints(3, 6));
  private class Capture<T> {
	public T inner;
	public Capture(T inner) {
		this.inner = inner;
	}
  }
  private TrapezoidProfile.State driveToPoseState = new State(0, 0);
  public Command driveToPoseC(Supplier<Pose2d> poseSupplier) {
	TrapezoidProfile.State state = new State(0, 0);
	Capture<Pose2d> start = new Capture<Pose2d>(new Pose2d());
	Capture<Pose2d> end = new Capture<Pose2d>(new Pose2d());
	Capture<Double> dist = new Capture<Double>(1.0);
	Timer time = new Timer();
	return runOnce(
		() -> {
		  m_profiledThetaController.reset(getPoseHeading().getRadians());
		  start.inner = getPose();
		  end.inner = poseSupplier.get();
		  dist.inner = end.inner.getTranslation().getDistance(start.inner.getTranslation());
		  state.position = dist.inner;
		  state.velocity = Math.min(0, -Pathing.velocityTowards(start.inner, getFieldRelativeLinearSpeedsMPS(), end.inner.getTranslation()));
		  time.reset();
		  time.start();
		}).andThen(run(()->{
		var setpoint = driveToPoseProfile.calculate(time.get(), state, driveToPoseState);
		var pose = end.inner;
		var curr = getPose();
		var startPose = start.inner;
		var poseTrans = pose.getTranslation();
		var currTrans = curr.getTranslation();
		
		var interp = end.inner.getTranslation().interpolate(startPose.getTranslation(), setpoint.position/dist.inner);
		driveFieldRelative(
			new ChassisSpeeds(
				m_xController.calculate(curr.getX(), interp.getX()),
				m_yController.calculate(curr.getY(), interp.getY()),
				m_profiledThetaController.calculate(curr.getRotation().getRadians(), pose.getRotation().getRadians()))
		);
	})).finallyDo(time::stop).alongWith(LightStripS.getInstance().stateC(()->States.AutoAlign));
  }




	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysId.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysId.dynamic(direction);
	}

	private static final Rotation2d BLUE_PERSPECTIVE = ZERO_ROTATION2D;
	private static final Rotation2d RED_PERSPECTIVE = new Rotation2d(Math.PI);
	public Pose2d getTargetPose() {
		return new Pose2d(
			m_xController.getSetpoint(),
			m_yController.getSetpoint(),
			new Rotation2d(
			m_thetaController.getSetpoint()
			)
		);
	}
	public boolean hasTarget() {
		return m_vision.hasTarget();
	}
	public void periodic() {
		m_vision.periodic();
		setOperatorPerspectiveForward(AllianceWrapper.isRed() ? RED_PERSPECTIVE : BLUE_PERSPECTIVE);
		if (true) {
		var swerveState = getState();

		
		for (int i = 0; i < Modules.length; i++) {
			var modLog = mods[i];
		
			var state = swerveState.ModuleStates[i];
			var target = swerveState.ModuleTargets[i];
			modLog.log("velocity", state.speedMetersPerSecond);
			modLog.log("tgtSpeed", target.speedMetersPerSecond);
			modLog.log("absVel", Math.abs(state.speedMetersPerSecond));
			modLog.log("absTgtSpeed", Math.abs(target.speedMetersPerSecond));
			modLog.log("angle", state.angle.getRadians());
			modLog.log("tgtAngle", target.angle.getRadians());
			var module = Modules[i];
			var drive = module.getDriveMotor();
			var steer = module.getSteerMotor();
			modLog.log("driveVolts", drive.getMotorVoltage().getValue());
			modLog.log("driveCurrent", drive.getSupplyCurrent().getValue());
			modLog.log("steerVolts", steer.getMotorVoltage().getValue());
			modLog.log("steerCurrent", steer.getSupplyCurrent().getValue());
		}}
	}

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return getState().speeds;
    }

  public void stop() {
	setControl(m_autoRequest.withSpeeds(ZERO_CHASSISSPEEDS));
  }
  public Command stopOnceC() {
	return runOnce(this::stop);
  }
  public Command stopC() {
	return run(this::stop);
  }
  public ChassisSpeeds getFieldRelativeLinearSpeedsMPS() {
	return ChassisSpeeds.fromRobotRelativeSpeeds(
		getRobotRelativeChassisSpeeds(), new Rotation2d(m_gyroYawRadsSupplier.getAsDouble()));
  }

  private Transform2d shotTransform = 
	new Transform2d(
		-10, 
		0, ZERO_ROTATION2D);
  public void drawRobotOnField(Field2d field) {
    field.setRobotPose(getPose());
    field.getObject("shot").setPoses(getPose(), getPose().transformBy(
      shotTransform)
      );
    // Draw a pose that is based on the robot pose, but shifted by the translation
    // of the module relative to robot center,
    // then rotated around its own center by the angle of the module.
    // Name starts with z so it draws on top on the field display
	var states = getState().ModuleStates;
    field
        .getObject("zmodules")
        .setPoses(
            List.of(
                getPose()
                    .transformBy(
                        new Transform2d(
						moduleConstants[FL].LocationX,
						moduleConstants[FL].LocationY,
                    	states[FL].angle
					)),
                getPose()
                  .transformBy(
                        new Transform2d(
						moduleConstants[FR].LocationX,
						moduleConstants[FR].LocationY,
                    	states[FR].angle
					)),
                getPose()
                  .transformBy(
                        new Transform2d(
						moduleConstants[BL].LocationX,
						moduleConstants[BL].LocationY,
                    	states[BL].angle
					)),
                getPose()
                    .transformBy(
                        new Transform2d(
						moduleConstants[BR].LocationX,
						moduleConstants[BR].LocationY,
                    	states[BR].angle
					))
		));
  }

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
				  double speed = MathUtil.applyDeadband(Math.hypot(fwdX, fwdY), 0.05) * MAX_LINEAR_SPEED;
				  double angle = Math.atan2(fwdY, fwdX);
                  

                  double rot = rotAxis.getAsDouble() * MAX_TURN_SPEED;
                  driveAllianceRelative(new ChassisSpeeds(speed * Math.cos(angle), speed* Math.sin(angle), rot));
                }));
  }
  public Command manualHeadingDriveC(
	InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleSupplier headingAllianceRelative) {
	  return manualHeadingDriveC(fwdXAxis, fwdYAxis, headingAllianceRelative, ()->0);
	}
  public Command manualHeadingDriveC(
	InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleSupplier headingAllianceRelative, DoubleSupplier headingFF) {
	  return manualFieldHeadingDriveC(fwdXAxis, fwdYAxis, ()->headingAllianceRelative.getAsDouble() + 
					((AllianceWrapper.getAlliance() == Alliance.Red) ? Math.PI : 0.0), headingFF);
	}
	public Rotation2d getPoseHeading() {
		return getPose().getRotation();
	  }
  public Command manualFieldHeadingDriveC(
	DoubleSupplier fwdXAxis, DoubleSupplier fwdYAxis, DoubleSupplier headingFieldRelative, DoubleSupplier headingFF) {
  return runOnce(
		  () -> {
			m_profiledThetaController.reset(getPoseHeading().getRadians());
		  })
	  .andThen(
		  run(
			  () -> {
				/**
				 * Units are given in meters per second radians per second Since joysticks give
				 * output from -1 to 1, we multiply the outputs by the max speed Otherwise, our
				 * max speed would be 1 meter per second and 1 radian per second
				 */
				double fwdX = fwdXAxis.getAsDouble() * MAX_LINEAR_SPEED;
				double fwdY = fwdYAxis.getAsDouble() * MAX_LINEAR_SPEED;
				double speed = MathUtil.applyDeadband(Math.hypot(fwdX, fwdY), 0.05);
				double angle = Math.atan2(fwdY, fwdX);
				double rot;

				rot =
					m_profiledThetaController.calculate(
						getPoseHeading().getRadians(),
						headingFieldRelative.getAsDouble());
				// log("thetaSetpt", m_profiledThetaController.getSetpoint().position);
				// log("thetaReal", getPoseHeading().getRadians());
				rot += headingFF.getAsDouble();
				driveAllianceRelative(new ChassisSpeeds(speed * Math.cos(angle), speed* Math.sin(angle), rot));
			  }));
			}
}
