package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.MoveCommand.MovementType;

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
        // This is if we are upstage and the team prop is upstage
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE && autoPosition.isUpstage) {
            addCommands(
                    // Driving to the correct position
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -18),
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-8)),
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(0)),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        // This is if we are downstage and the team prop is upstage
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(7)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -4),
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 3),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -7),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -4),
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(-90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -2),
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
