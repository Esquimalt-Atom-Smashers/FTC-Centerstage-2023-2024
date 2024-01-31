package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command that drives from where we sensed the distance sensors to the correct spot to place the
 * purple pixel, then places the purple pixel.
 */
public class AutoDriveAndPlacePurpleCommand extends SequentialCommandGroup {
    private final Command lastCommand;

    /**
     * Creates the command that drives from where we sensed the distance sensors to the correct spot to place the
     * purple pixel, then places the purple pixel.
     *
     * @param driveSubsystem A reference to the driveSubsystem
     * @param intakeSubsystem A reference to the intakeSubsystem
     * @param autoPosition The auto position we started at
     */
    public AutoDriveAndPlacePurpleCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, AutoPosition autoPosition) {
        int multiplier = autoPosition.isBlue ? 1 : -1;
        // This is if we are upstage and the team prop is upstage
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE && autoPosition.isUpstage) {
            addCommands(
                    // Driving to the correct position
                    new DriveCommand(driveSubsystem, -18),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, -8 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0 * multiplier),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        // This is if we are downstage and the team prop is upstage
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE) {
            addCommands(
                    new StrafeCommand(driveSubsystem, 7 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, 90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 3),
                    new WaitCommand(250),
                    // Placing purple
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -7),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, -90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -2),
                    new WaitCommand(250),
                    // Placing purple
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        lastCommand = new WaitCommand(1);
        addCommands(lastCommand);
        addRequirements(driveSubsystem, intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
