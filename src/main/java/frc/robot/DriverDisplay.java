package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class DriverDisplay {
    private DoublePublisher matchTimeEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("/DriverDisplay/matchTime").publish();
    private BooleanPublisher hasNoteEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/hasNote").publish();
    private BooleanSupplier hasNoteSupplier = ()->false;
    public void setHasNoteSupplier(BooleanSupplier hasNoteSupplier) {
        this.hasNoteSupplier = hasNoteSupplier;
    }

    private BooleanPublisher inRangeEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inRange").publish();
    private BooleanSupplier inRangeSupplier = ()->false;
    public void setInRangeSupplier(BooleanSupplier inRangeSupplier) {
        this.inRangeSupplier = inRangeSupplier;
    }

    private BooleanPublisher inAngleEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inAngle").publish();
    private BooleanSupplier inAngleSupplier = ()->false;
    public void setInAngleSupplier(BooleanSupplier inAngleSupplier) {
        this.inAngleSupplier = inAngleSupplier;
    }

    private BooleanPublisher inPivotEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/inPivot").publish();
    private BooleanSupplier inPivotSupplier = ()->false;
    public void setInPivotSupplier(BooleanSupplier inPivotSupplier) {
        this.inPivotSupplier = inPivotSupplier;
    }

    public DriverDisplay() {}
    
    public void update() {
        matchTimeEntry.accept(DriverStation.getMatchTime());
        hasNoteEntry.accept(hasNoteSupplier.getAsBoolean());
        inRangeEntry.accept(inRangeSupplier.getAsBoolean());
        inAngleEntry.accept(inAngleSupplier.getAsBoolean());
        inPivotEntry.accept(inPivotSupplier.getAsBoolean());
    }
}
