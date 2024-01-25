package org.firstinspires.ftc.teamcode.auto;

public class AutoPosition {
    public NewAutonomousController.SpikeMark spikeMark;
    public boolean isBlue;
    public boolean isPlacingYellow;
    public boolean isUpstage;

    public AutoPosition(boolean isBlue, boolean isPlacingYellow, boolean isUpstage) {
        this.isBlue = isBlue;
        this.isPlacingYellow = isPlacingYellow;
        this.isUpstage = isUpstage;
    }

    public AutoPosition(NewAutonomousController.SpikeMark spikeMark, boolean isBlue, boolean isPlacingYellow, boolean isUpstage) {
        this(isBlue, isPlacingYellow, isUpstage);
        this.spikeMark = spikeMark;
    }

    public void setSpikeMark(NewAutonomousController.SpikeMark spikeMark) {
        this.spikeMark = spikeMark;
    }
}
