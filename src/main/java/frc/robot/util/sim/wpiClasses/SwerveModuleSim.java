package frc.robot.util.sim.wpiClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveModuleSim {

  private SimpleMotorWithMassModel azmthMotor;
  private MotorGearboxWheelSim wheelMotor;

  private final double azimuthEncGearRatio; // Motor-to-azimuth-encoder reduction
  private final double wheelEncGearRatio; // Motor-to-wheel-encoder reduction
  private final double treadStaticFricForce;
  private final double treadKineticFricForce;
  private final double wheelGearboxLossFactor = 0.01;
  Pose2d prevModulePose = null;
  Pose2d curModulePose = null;
  double curLinearSpeed_mps = 0; // Positive = in curAngle_deg, Negative = opposite of curAngle_deg
  Rotation2d curAzmthAngle =
      Rotation2d.fromDegrees(
          0); // 0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

  double crossTreadFricForceMag = 0;
  double crossTreadVelMag = 0;
  double crossTreadForceMag = 0;

  double wheelVoltage;
  double azmthVoltage;

  public SwerveModuleSim(
      DCMotor azimuthMotor,
      DCMotor wheelMotor,
      double wheelRadius_m,
      double azimuthGearRatio, // Motor rotations per one azimuth module rotation. Should be greater
      // than zero
      double wheelGearRatio, // Motor rotations per one wheel rotation. Should be greater than zero
      double
          azimuthEncGearRatio, // Encoder rotations per one azimuth module rotation. Should be 1.0
      // if you have a good swerve module.
      double wheelEncGearRatio, // Encoder rotations per one wheel rotation.
      double treadStaticCoefFric,
      double treadKineticCoefFric,
      double moduleNormalForce,
      double azimuthEffectiveMOI) {
    this.azmthMotor =
        new SimpleMotorWithMassModel(azimuthMotor, azimuthGearRatio, azimuthEffectiveMOI);
    this.wheelMotor =
        new MotorGearboxWheelSim(wheelMotor, wheelGearRatio, wheelRadius_m, wheelGearboxLossFactor);

    this.azimuthEncGearRatio = azimuthEncGearRatio;
    this.wheelEncGearRatio = wheelEncGearRatio;
    this.treadStaticFricForce = treadStaticCoefFric * moduleNormalForce;
    this.treadKineticFricForce = treadKineticCoefFric * moduleNormalForce;
  }

  public void setInputVoltages(double wheelVoltage, double azmthVoltage) {
    this.wheelVoltage = wheelVoltage;
    this.azmthVoltage = azmthVoltage;
  }

  public void setWheelVoltage(double wheelVoltage) {
    this.wheelVoltage = wheelVoltage;
  }

  public double getWheelVoltage() {
    return this.wheelVoltage;
  }

  public void setAzmthVoltage(double azmthVoltage) {
    this.azmthVoltage = azmthVoltage;
  }

  public double getAzmthVoltage() {
    return this.azmthVoltage;
  }

  public double getAzimuthEncoderPositionRev() {
    return azmthMotor.getMechanismPosition_Rev() * azimuthEncGearRatio;
  }

  public double getWheelEncoderPositionRev() {
    return wheelMotor.getPosition_Rev() * wheelEncGearRatio;
  }

  public double getWheelEncoderVelocityRevPerSec() {
    return wheelMotor.getVelocity_RevPerSec() * wheelEncGearRatio;
  }

  void reset(Pose2d initModulePose) {
    prevModulePose = curModulePose = initModulePose;
    curLinearSpeed_mps = 0;
    curAzmthAngle = Rotation2d.fromDegrees(0);
  }

  public void resetAzmth(double angleRad) {
    azmthMotor.setPosition_Rev(angleRad / 2.0 / Math.PI);
  }

  public void resetWheel(double positionRad) {
    wheelMotor.setPosition_Rev(positionRad / 2.0 / Math.PI);
  }

  void update(double dtSeconds) {

    Vector2d azimuthUnitVec = new Vector2d(1, 0);
    azimuthUnitVec.rotate(curAzmthAngle.getDegrees());

    // Assume the wheel does not lose traction along its wheel direction (on-tread)
    double velocityAlongAzimuth =
        getModuleRelativeTranslationVelocity(dtSeconds).dot(azimuthUnitVec);

    wheelMotor.update(velocityAlongAzimuth, wheelVoltage, dtSeconds);
    azmthMotor.update(azmthVoltage, dtSeconds);

    // Assume idealized azimuth control - no "twist" force at contact patch from friction or robot
    // motion.
    curAzmthAngle = Rotation2d.fromDegrees(azmthMotor.getMechanismPosition_Rev() * 360);
  }

  /** Get a vector of the velocity of the module's contact patch moving across the field. */
  Vector2d getModuleRelativeTranslationVelocity(double dtSeconds) {
    double xVel =
        (curModulePose.getTranslation().getX() - prevModulePose.getTranslation().getX())
            / dtSeconds;
    double yVel =
        (curModulePose.getTranslation().getY() - prevModulePose.getTranslation().getY())
            / dtSeconds;
    Vector2d moduleTranslationVec = new Vector2d(xVel, yVel);
    moduleTranslationVec.rotate(-1.0 * curModulePose.getRotation().getDegrees());
    return moduleTranslationVec;
  }

  /**
   * Given a net force on a particular module, calculate the friction force generated by the tread
   * interacting with the ground in the direction perpendicular to the wheel's rotation.
   *
   * @param netForce_in
   * @return
   */
  ForceAtPose2d getCrossTreadFrictionalForce(Force2d netForce_in, double dtSeconds) {

    // Project net force onto cross-tread vector
    Vector2d crossTreadUnitVector = new Vector2d(0, 1);
    crossTreadUnitVector.rotate(curAzmthAngle.getDegrees());
    crossTreadVelMag = getModuleRelativeTranslationVelocity(dtSeconds).dot(crossTreadUnitVector);
    crossTreadForceMag = netForce_in.getVector2d().dot(crossTreadUnitVector);

    Force2d fricForce = new Force2d();

    if (Math.abs(crossTreadForceMag) > treadStaticFricForce || Math.abs(crossTreadVelMag) > 0.001) {
      // Force is great enough to overcome static friction, or we're already moving
      // In either case, use kinetic frictional model
      crossTreadFricForceMag = -1.0 * Math.signum(crossTreadVelMag) * treadKineticFricForce;
    } else {
      // Static Friction Model
      crossTreadFricForceMag = -1.0 * crossTreadForceMag;
    }

    fricForce = new Force2d(crossTreadUnitVector);
    fricForce = fricForce.times(crossTreadFricForceMag);

    return new ForceAtPose2d(fricForce, curModulePose);
  }

  /**
   * Gets the modules on-axis (along wheel direction) force, which comes from the rotation of the
   * motor.
   */
  ForceAtPose2d getWheelMotiveForce() {
    return new ForceAtPose2d(
        new Force2d(wheelMotor.getGroundForce_N(), curAzmthAngle), curModulePose);
  }

  /** Set the motion of each module in the field reference frame */
  void setModulePose(Pose2d curPos) {
    // Handle init'ing module position history to current on first pass
    if (prevModulePose == null) {
      prevModulePose = curPos;
    } else {
      prevModulePose = curModulePose;
    }

    curModulePose = curPos;
  }

  Pose2d getModulePose() {
    return curModulePose;
  }
}
