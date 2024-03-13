package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.MoveCommand.MovementType;

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
        // If we don't want to place the yellow, just turn to face the correct direction, then stop
        if (!autoPosition.isPlacingYellow) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, 0),
                    new MoveCommand(driveSubsystem, MovementType.SLOW_TURN_TO_HEADING, 0)
            );
            return;
        }

        if (autoPosition.isUpstage) {
            placingUpstage(driveSubsystem, autoPosition);
        }
        else {
            placingDownstage(driveSubsystem, autoPosition);
        }
    }

    private void placingUpstage(DriveSubsystem driveSubsystem, AutoPosition autoPosition) {
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 25),
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(8))
            );
        } else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 32),
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(6)),
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90))
            );
        } else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -4),
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(180)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 26)
            );
        }
    }

    private void placingDownstage(DriveSubsystem driveSubsystem, AutoPosition autoPosition) {
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE) {
            // We start around 28 inches forwards, 4 inches right, facing the backdrop
            // TODO: Be super careful testing this!!!
            addCommands(
                    // Strafe a bit
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(24)),
                    // Turn to heading
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90)),
                    // Drive forwards
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 80),
                    // Strafe
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-25)),
                    // Turn to heading
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90))
            );
        } else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE) {
            // We start 25 inches forwards
            // TODO: Be super careful testing this!!!
            addCommands(
                    // Move forwards a bit
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 25),
                    // Rotate to 90
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(90)),
                    // Turn to heading to be safe?
                    new MoveCommand(driveSubsystem, MovementType.SLOW_TURN_TO_HEADING, autoPosition.flip(90)),
                    // Drive forwards under the truss, stopping at the tape
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 80),
                    // Strafe left a bit
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-23)),
                    // Turn to heading again
                    new MoveCommand(driveSubsystem, MovementType.SLOW_TURN_TO_HEADING, autoPosition.flip(90))
                    // End
            );
        } else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE) {
            // We start about 28 inches forwards, 2 inches left
            // TODO: Be super careful testing this!!!
            addCommands(
                    // Strafe right
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, 23),
                    // Turn to heading
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90)),
                    // Drive forwards
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, 75),
                    // Turn to heading
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90)),
                    // Strafe left
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-24)),
                    // Turn to heading
                    new MoveCommand(driveSubsystem, MovementType.SLOW_TURN_TO_HEADING, autoPosition.flip(90))

            );
        }
    }
}