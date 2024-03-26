package frc.robot.subsystems.drive;

import monologue.Logged;

public class SwerveModuleLog implements Logged {
    private String name;
    public SwerveModuleLog(String name) {
this.name = name;
    }
    @Override
    public String getPath() {
        return name;
    }

}
