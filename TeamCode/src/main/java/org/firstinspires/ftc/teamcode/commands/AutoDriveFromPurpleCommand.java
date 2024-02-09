package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command that drives from where we placed the purple pixel to the next spot,
 * either the backdrop or just facing the correct direction.
 */
public class AutoDriveFromPurpleCommand extends SequentialCommandGroup {

    /**
     * Creates a command that drives from where we placed the purple pixel to the next spot,
     * either the backdrop or just facing the correct direction.
     *
     * @param driveSubsystem A reference to the driveSubsystem
     * @param autoPosition The starting auto position
     */
    public AutoDriveFromPurpleCommand(DriveSubsystem driveSubsystem, AutoPosition autoPosition) {
        if (!autoPosition.isPlacingYellow) {
            addCommands(
                    new TurnToHeadingCommand(driveSubsystem, 0),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0, Constants.DriveConstants.AUTO_TURN_SPEED/2)
            );
            return;
        }
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.TURN, autoPosition.flip(90)),
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.DRIVE, 25),
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.STRAFE, autoPosition.flip(8))
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.TURN, autoPosition.flip(90)),
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.DRIVE, 32),
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.STRAFE, autoPosition.flip(6)),
                    new TurnToHeadingCommand(driveSubsystem, autoPosition.flip(90))
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.DRIVE, -4),
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.TURN, autoPosition.flip(180)),
                    new MoveCommand(driveSubsystem, MoveCommand.MovementType.DRIVE, 26)
            );
        }
    }
}