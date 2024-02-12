package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import java.util.ArrayList;
import java.util.function.Consumer;

public class RealSwerveDriveIO extends SwerveDriveIO {
  public RealSwerveDriveIO(Consumer<Runnable> addPeriodic) {
    super(addPeriodic);
    m_modules = new ArrayList<>();
    //Timer.delay(0.05);
    m_modules.add(new RealModuleIO(addPeriodic, ModuleConstants.FL));
    //Timer.delay(0.05);
    m_modules.add(new RealModuleIO(addPeriodic, ModuleConstants.FR));
    //Timer.delay(0.05);
    m_modules.add(new RealModuleIO(addPeriodic, ModuleConstants.BL));
    //Timer.delay(0.05);
    m_modules.add(new RealModuleIO(addPeriodic, ModuleConstants.BR));
  }

  public void resetPose(Pose2d pose) {}
  ;

  @Override
  public void resetIMU() {
    m_navx.reset();
  }

  @Override
  public Pose2d getSimPose() {
    return new Pose2d();
  }
}
