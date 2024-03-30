package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class DriverDisplay {
    // private DoublePublisher matchTimeEntry =
    //     NetworkTableInstance.getDefault().getDoubleTopic("/DriverDisplay/matchTime").publish();
    // private DoublePublisher maxTimeEntry =
    //     NetworkTableInstance.getDefault().getDoubleTopic("/DriverDisplay/maxTime").publish();
    // private BooleanPublisher hasNoteEntry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/hasNote").publish();
    private BooleanSupplier hasNoteSupplier = ()->false;
    public void setHasNoteSupplier(BooleanSupplier hasNoteSupplier) {
        this.hasNoteSupplier = hasNoteSupplier;
    }

    // private BooleanPublisher inRangeEntry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inRange").publish();
    private BooleanSupplier inRangeSupplier = ()->false;
    public void setInRangeSupplier(BooleanSupplier inRangeSupplier) {
        this.inRangeSupplier = inRangeSupplier;
    }

    // private BooleanPublisher inAngleEntry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inAngle").publish();
    private BooleanSupplier inAngleSupplier = ()->false;
    public void setInAngleSupplier(BooleanSupplier inAngleSupplier) {
        this.inAngleSupplier = inAngleSupplier;
    }

    // private BooleanPublisher inPivotEntry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inPivot").publish();
    private BooleanSupplier inPivotSupplier = ()->false;
    public void setInPivotSupplier(BooleanSupplier inPivotSupplier) {
        this.inPivotSupplier = inPivotSupplier;
    }

    // private BooleanPublisher inSpeedEntry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inSpeed").publish();
    private BooleanSupplier inSpeedSupplier = ()->false;
    public void setInSpeedSupplier(BooleanSupplier inSpeedSupplier) {
        this.inSpeedSupplier = inSpeedSupplier;
    }

    // private BooleanPublisher seeNoteEntry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/seeNote").publish();
    private BooleanSupplier seeNoteSupplier = ()->false;
    public void setSeeNoteSupplier(BooleanSupplier seeNoteSupplier) {
        this.seeNoteSupplier = seeNoteSupplier;
    }

    private BooleanPublisher intakeHomedEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/intakeHomed").publish();
    private BooleanSupplier intakeHomedSupplier = ()->false;
    public void setIntakeHomedSupplier(BooleanSupplier seeNoteSupplier) {
        this.intakeHomedSupplier = seeNoteSupplier;
    }

    // private BooleanPublisher controller0Entry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/controller0").publish();
    // private BooleanPublisher controller1Entry =
    //     NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/controller1").publish();
    public DriverDisplay() {}
    
    public void update() {
        // maxTimeEntry.accept(DriverStation.isAutonomous()?15:135);
        // matchTimeEntry.accept(DriverStation.getMatchTime());
        
        // hasNoteEntry.accept(hasNoteSupplier.getAsBoolean());
        // inRangeEntry.accept(inRangeSupplier.getAsBoolean());
        // inAngleEntry.accept(inAngleSupplier.getAsBoolean());
        // inPivotEntry.accept(inPivotSupplier.getAsBoolean());
        // inSpeedEntry.accept(inSpeedSupplier.getAsBoolean());
        // seeNoteEntry.accept(seeNoteSupplier.getAsBoolean());
        intakeHomedEntry.accept(intakeHomedSupplier.getAsBoolean());
        // controller0Entry.accept(DriverStation.isJoystickConnected(0));
        // controller1Entry.accept(DriverStation.isJoystickConnected(1));
    }
}
