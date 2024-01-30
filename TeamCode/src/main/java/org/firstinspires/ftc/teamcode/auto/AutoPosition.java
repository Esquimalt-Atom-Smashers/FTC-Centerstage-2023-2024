package org.firstinspires.ftc.teamcode.auto;

/**
 * Represents all of the variables for an auto position. Contains whether we are on the blue
 * alliance, whether we want to place a yellow, whether we are upstage (closer to the backdrop), and
 * where the spike mark is.
 */
public class AutoPosition {
    public enum SpikeMark {
        UPSTAGE,
        MIDDLE,
        DOWNSTAGE
    }

    public SpikeMark spikeMark;
    public boolean isBlue;
    public boolean isPlacingYellow;
    public boolean isUpstage;

    public AutoPosition(boolean isBlue, boolean isPlacingYellow, boolean isUpstage) {
        this.isBlue = isBlue;
        this.isPlacingYellow = isPlacingYellow;
        this.isUpstage = isUpstage;
    }

    public AutoPosition(SpikeMark spikeMark, boolean isBlue, boolean isPlacingYellow, boolean isUpstage) {
        this(isBlue, isPlacingYellow, isUpstage);
        this.spikeMark = spikeMark;
    }

    public void setSpikeMark(SpikeMark spikeMark) {
        this.spikeMark = spikeMark;
    }
}
