package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.vision.CTREVision;
import frc.robot.subsystems.vision.CTREVision.VisionMeasurement;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.trajectory.PPChasePoseCommand;
import monologue.Logged;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import java.util.List;

import java.util.function.BiConsumer;
import static frc.robot.generated.TunerConstants.kDriveRadius;
import static frc.robot.generated.TunerConstants.kDriveRotationsPerMeter;
import static edu.wpi.first.units.Units.Volts;

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
	
	private Rotation2d m_desiredRot = new Rotation2d();

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

	// private final DoubleLogger log_accumGyro = WaltLogger.logDouble("Swerve", "accumGyro");
	// private final DoubleLogger log_avgWheelPos = WaltLogger.logDouble("Swerve", "avgWheelPos");
	// private final DoubleLogger log_curEffWheelRad = WaltLogger.logDouble("Swerve", "curEffWheelRad");
	// private final DoubleLogger log_lastGyro = WaltLogger.logDouble("Swerve", "lastGyro");
	// private final DoubleLogger log_rotationSpeed = WaltLogger.logDouble("Swerve", "rot_sec",
	// 	PubSubOption.sendAll(true));

	// private final DoubleLogger log_desiredRot = WaltLogger.logDouble("Swerve", "desiredRot");
	// private final DoubleLogger log_rot = WaltLogger.logDouble("Swerve", "rotation");
	// private final DoubleArrayLogger log_poseError = WaltLogger.logDoubleArray("Swerve", "poseError");
	private double[] m_poseError = new double[3];
	//private final DoubleArrayLogger log_desiredPose = WaltLogger.logDoubleArray("Swerve", "desiredPose");
	private double[] m_desiredPose = new double[3];
	//private final DoubleArrayLogger log_wheelVeloErrors = WaltLogger.logDoubleArray("Swerve", "wheelVeloErrors");
	private double[] m_wheelVeloErrs = new double[4];
	//private final DoubleArrayLogger log_wheelVelos = WaltLogger.logDoubleArray("Swerve", "wheelVelos");
	private double[] m_wheelVelos = new double[4];
	//private final DoubleArrayLogger log_wheelVeloTargets = WaltLogger.logDoubleArray("Swerve", "wheelVeloTargets");
	private double[] m_wheelVeloTargets = new double[4];


	public void addVisionMeasurement(VisionMeasurement measurement) {
		addVisionMeasurement(
			measurement.pose(), measurement.timestamp(), measurement.stddevs());
	}
	private void configureAutoBuilder() {
		AutoBuilder.configureHolonomic(
			() -> getState().Pose,
			this::seedFieldRelative,
			() -> m_kinematics.toChassisSpeeds(getState().ModuleStates),
			(speeds) -> setControl(m_autoRequest.withSpeeds(speeds)),
			Pathing.m_pathPlannerConfig,
			AllianceWrapper::isRed,
			this);
	}

	public Swerve( SwerveDrivetrainConstants driveTrainConstants, double odometryFreq, SwerveModuleConstants... modules) {
		super(driveTrainConstants, odometryFreq, modules);
		this.moduleConstants = modules;
        m_vision = new CTREVision(this::addVisionMeasurement, this::getPose);
		configureAutoBuilder();
		if (Utils.isSimulation()) {
			startSimThread();
		}
		m_gyroYawRadsSupplier = () -> Units.degreesToRadians(getPigeon2().getAngle());
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
				double averageWheelPosition = 0;
				double[] wheelPositions = new double[4];
				for (int i = 0; i < Modules.length; i++) {
					var pos = Modules[i].getPosition(true);
					wheelPositions[i] = pos.distanceMeters * kDriveRotationsPerMeter;
					averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
				}
				averageWheelPosition /= 4.0;
				currentEffectiveWheelRadius = (accumGyroYawRads * kDriveRadius) / averageWheelPosition;
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
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
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

	// public Command resetPoseToSpeaker() {
	// 	return runOnce(() -> {
	// 		if (DriverStation.getAlliance().get() == Alliance.Blue) {
	// 			seedFieldRelative(new Pose2d(1.45, 5.5, Rotation2d.fromRadians(0)));
	// 		} else {
	// 			seedFieldRelative(new Pose2d(1.45, kFieldWidth.magnitude() - 5.5, Rotation2d.fromRadians(0)));
	// 		}
	// 	});
	// }

	// public Command goToAutonPose() {
	// 	return run(() -> {
	// 		var bluePose = AutonChooser.getChosenAutonInitPose();
	// 		if (bluePose.isPresent()) {
	// 			Pose2d pose;
	// 			if (DriverStation.getAlliance().get() == Alliance.Red) {
	// 				Translation2d redTranslation = new Translation2d(bluePose.get().getX(),
	// 					kFieldWidth.magnitude() - bluePose.get().getY());
	// 				Rotation2d redRotation = bluePose.get().getRotation().times(-1);
	// 				pose = new Pose2d(redTranslation, redRotation);
	// 			} else {
	// 				pose = bluePose.get();
	// 			}

	// 			SmartDashboard.putNumberArray("desiredPose", AdvantageScopeUtil.toDoubleArr(pose));

	// 			var curPose = getState().Pose;
	// 			var xSpeed = m_xController.calculate(curPose.getX(), pose.getX());
	// 			var ySpeed = m_yController.calculate(curPose.getY(), pose.getY());
	// 			var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(),
	// 				pose.getRotation().getRadians());
	// 			var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, pose.getRotation());

	// 			setControl(m_autoRequest.withSpeeds(speeds));
	// 		}
	// 	});
	// }

	// public Command aim(double radians) {
	// 	return run(() -> {
	// 		m_desiredRot = AllianceFlipUtil.apply(Rotation2d.fromRadians(radians));
	// 		var curPose = getState().Pose;
	// 		var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(),
	// 			m_desiredRot.getRadians());
	// 		var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaSpeed, m_desiredRot);

	// 		setControl(m_autoRequest.withSpeeds(speeds));
	// 	}).until(() -> {
	// 		boolean check = MathUtil.isNear(m_desiredRot.getDegrees(), getState().Pose.getRotation().getDegrees(), 1);
	// 		if (check) {
	// 		}
	// 		return check;
	// 	});
	// }

	public Pose3d getPose3d() {
		var txr2d = getState().Pose.getTranslation();
		// we're on the floor. I hope. (i'm going to make the robot fly! >:D)
		return new Pose3d(txr2d.getX(), txr2d.getY(), 0, getRotation3d());
	}

  public Command resetPoseToBeginningC(PathPlannerTrajectory trajectory) {
    return Commands.runOnce(
        () ->
            seedFieldRelative(
                NomadMathUtil.mirrorPose(
                    new Pose2d(
                        trajectory.getInitialState().positionMeters,
                        trajectory.getInitialState().targetHolonomicRotation),
                    AllianceWrapper.getAlliance())));
  }


	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysId.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysId.dynamic(direction);
	}

	private static final Rotation2d BLUE_PERSPECTIVE = new Rotation2d();
	private static final Rotation2d RED_PERSPECTIVE = new Rotation2d(Math.PI);
	public void periodic() {
		m_vision.periodic();
		setOperatorPerspectiveForward(AllianceWrapper.isRed() ? RED_PERSPECTIVE : BLUE_PERSPECTIVE);
		var swerveState = getState();
		log("rotationSpeed", swerveState.speeds.omegaRadiansPerSecond);
		// log_rotationSpeed.accept(Units.radiansToRotations(swerveState.speeds.omegaRadiansPerSecond));
		// log_desiredRot.accept(m_desiredRot.getDegrees());
		// log_rot.accept(swerveState.Pose.getRotation().getDegrees());
		// m_poseError[0] = m_xController.getPositionError();
		// m_poseError[1] = m_yController.getPositionError();
		// m_poseError[2] = Units.radiansToDegrees(m_thetaController.getPositionError());
		//log_poseError.accept(m_poseError);
		// m_desiredPose[0] = m_xController.getSetpoint();
		// m_desiredPose[1] = m_yController.getSetpoint();
		// m_desiredPose[2] = Units.radiansToDegrees(m_thetaController.getSetpoint());
		// log_desiredPose.accept(m_desiredPose);

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
			
			
			// m_wheelVelos[i] = Math.abs(swerveState.ModuleStates[i].speedMetersPerSecond);
			// m_wheelVeloTargets[i] = Math.abs(swerveState.ModuleTargets[i].speedMetersPerSecond);
			// m_wheelVeloErrs[i] = Math.abs(m_wheelVeloTargets[i] - m_wheelVelos[i]);
		}
		// log_wheelVelos.accept(m_wheelVelos);
		// log_wheelVeloTargets.accept(m_wheelVeloTargets);
		// log_wheelVeloErrors.accept(m_wheelVeloErrs);
	}

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return getState().speeds;
    }

    public Command chasePoseC(Supplier<Pose2d> targetSupplier) {
    return new PPChasePoseCommand(
            targetSupplier,
            this::getPose,
            Pathing.m_holonomicDriveController,
            m_xController,
            m_yController,
            m_thetaController,
            (speeds)->setControl(m_autoRequest.withSpeeds(speeds)),
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
  public void stop() {
	setControl(m_autoRequest.withSpeeds(new ChassisSpeeds()));
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
		-10 * Math.cos(Units.degreesToRadians(-6)), 
		-10* Math.sin(Units.degreesToRadians(-6)), new Rotation2d());
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
                  double fwdX = fwdXAxis.getAsDouble() * MAX_LINEAR_SPEED;
                  double fwdY = fwdYAxis.getAsDouble() * MAX_LINEAR_SPEED;
                  double rot = rotAxis.getAsDouble() * MAX_TURN_SPEED;
                  driveAllianceRelative(new ChassisSpeeds(fwdX, fwdY, rot));
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
	InputAxis fwdXAxis, InputAxis fwdYAxis, DoubleSupplier headingFieldRelative, DoubleSupplier headingFF) {
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
				log("thetaSetpt", m_profiledThetaController.getSetpoint().position);
				log("thetaReal", getPoseHeading().getRadians());
				rot += headingFF.getAsDouble();
				driveAllianceRelative(new ChassisSpeeds(fwdX, fwdY, rot));
			  }));
			}
}
