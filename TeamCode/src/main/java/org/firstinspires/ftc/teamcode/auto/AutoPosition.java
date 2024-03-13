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

    /** Which spike mark the team prop is on. */
    public SpikeMark spikeMark;
    /** If we are on the blue alliance. */
    public boolean isBlue;
    /** Whether we want to place the yellow on the backdrop. */
    public boolean isPlacingYellow;
    /** Whether we are upstage. */
    public boolean isUpstage;

    /**
     * Creates an autonomous position with the specified parameters. Does not set what the spike mark position is.
     *
     * @param isBlue If we are on the blue alliance.
     * @param isPlacingYellow Whether we want to place the yellow on the backdrop.
     * @param isUpstage Whether we are upstage.
     */
    public AutoPosition(boolean isBlue, boolean isPlacingYellow, boolean isUpstage) {
        this.isBlue = isBlue;
        this.isPlacingYellow = isPlacingYellow;
        this.isUpstage = isUpstage;
    }

    /**
     * Creates an autonomous position with the specified parameters.
     *
     * @param spikeMark Which spike mark the team prop is on.
     * @param isBlue If we are on the blue alliance.
     * @param isPlacingYellow Whether we want to place the yellow on the backdrop.
     * @param isUpstage Whether we are upstage.
     */
    public AutoPosition(SpikeMark spikeMark, boolean isBlue, boolean isPlacingYellow, boolean isUpstage) {
        this(isBlue, isPlacingYellow, isUpstage);
        this.spikeMark = spikeMark;
    }

    /**
     * Sets the spike mark to the specified spike mark.
     *
     * @param spikeMark The spike mark the team prop is on.
     */
    public void setSpikeMark(SpikeMark spikeMark) {
        this.spikeMark = spikeMark;
    }

    /** @return The input negated if we are red, or the input if we are blue */
    public double flip(double input) {
        return isBlue ? input : -input;
    }
}
