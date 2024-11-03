package frc.robot.subsystems.drive;

import monologue.Logged;

public class SwerveModuleLog implements Logged {
    private String name;
    public final double offsetRots; 
    public SwerveModuleLog(String name, double offsetRots) {
this.name = name;
this.offsetRots = offsetRots;
    }
    @Override
    public String getPath() {
        return name;
    }

}
