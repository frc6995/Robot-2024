package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.sim.SimGyroSensorModel;
import frc.robot.util.sim.wpiClasses.QuadSwerveSim;
import frc.robot.util.sim.wpiClasses.SwerveModuleSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class SimSwerveDriveIO extends SwerveDriveIO {

  private final List<SwerveModuleSim> m_moduleSims;
  private final QuadSwerveSim m_quadSwerveSim;
  private SimGyroSensorModel m_simNavx = new SimGyroSensorModel();

  public SimSwerveDriveIO(Consumer<Runnable> addPeriodic) {
    super(addPeriodic);
    m_modules =
        List.of(
            new SimModuleIO(addPeriodic, ModuleConstants.FL),
            new SimModuleIO(addPeriodic, ModuleConstants.FR),
            new SimModuleIO(addPeriodic, ModuleConstants.BL),
            new SimModuleIO(addPeriodic, ModuleConstants.BR));
    m_moduleSims = new ArrayList<SwerveModuleSim>();
    for (ModuleIO module : m_modules) {
      if (module instanceof SimModuleIO) {
        m_moduleSims.add(((SimModuleIO) module).getModuleSim());
      }
    }
    m_quadSwerveSim =
        new QuadSwerveSim(
            WHEEL_BASE_WIDTH_M, WHEEL_BASE_WIDTH_M, ROBOT_MASS_kg, ROBOT_MOI_KGM2, m_moduleSims);
    addPeriodic.accept(this::periodic);
  }

  private void periodic() {

    Pose2d prevRobotPose = m_quadSwerveSim.getCurPose();

    // Update model (several small steps)
    for (int i = 0; i < 40; i++) {
      m_quadSwerveSim.update(0.0005);
    }

    // Set the gyro based on the difference between the previous pose and this pose.
    m_simNavx.update(m_quadSwerveSim.getCurPose(), prevRobotPose);
  }

  public void resetPose(Pose2d pose) {
    m_quadSwerveSim.modelReset(pose);
  }
  ;

  @Override
  public Rotation2d getGyroHeading() {
    return m_simNavx.getRotation2d();
  }

  @Override
  public void resetIMU() {
    m_simNavx.resetToPose(new Pose2d());
  }

  @Override
  public Pose2d getSimPose() {
    return m_quadSwerveSim.getCurPose();
  }
}
