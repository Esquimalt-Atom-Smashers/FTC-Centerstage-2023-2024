package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnToHeadingCommand extends TurnByAngleCommand {
    private final double targetHeading;

    public TurnToHeadingCommand(DriveSubsystem driveSubsystem, double targetHeading, double speed) {
        // Just call the super constructor to add requirement and things
        // We will set the angle when the command is actually initialized
        super(driveSubsystem, 0, speed);
        this.targetHeading = targetHeading;
    }

    public TurnToHeadingCommand(DriveSubsystem driveSubsystem, double heading) {
        this(driveSubsystem, heading, Constants.DriveConstants.AUTO_TURN_SPEED);
    }

    // Initialize is called when the command is about to begin, so we
    // set the angle here
    @Override
    public void initialize() {
        angle = targetHeading - driveSubsystem.getHeading();
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
