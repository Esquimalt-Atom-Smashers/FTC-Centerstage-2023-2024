package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command that uses the driveSubsystem to turn to a heading.
 */
public class TurnToHeadingCommand extends TurnByAngleCommand {
    private final double targetHeading;

    /**
     * Creates a command that rotates the robot to a specified heading at a specified speed.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param targetHeading The target heading in degrees
     * @param speed The speed to turn at
     */
    public TurnToHeadingCommand(DriveSubsystem driveSubsystem, double targetHeading, double speed) {
        // Just call the super constructor to add requirement and things
        // We will set the angle when the command is actually initialized
        super(driveSubsystem, 0, speed);
        this.targetHeading = targetHeading;
    }

    /**
     * Creates a command that rotates the robot to a specified heading at a specified speed.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param targetHeading The target heading in degrees
     */
    public TurnToHeadingCommand(DriveSubsystem driveSubsystem, double targetHeading) {
        this(driveSubsystem, targetHeading, Constants.DriveConstants.AUTO_TURN_SPEED);
    }

    // Initialize is called when the command is about to begin, so we
    // set the angle here
    @Override
    public void initialize() {
        angle = targetHeading - driveSubsystem.getHeading();
        super.initialize();
    }
}
