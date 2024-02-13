package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.commands.MoveCommand.MovementType;

/**
 * Command that drives to the correct position on the backdrop, places the yellow pixel on the (hopefully)
 * correct position, and drives into the corner.
 */
public class AutoPlaceYellowAndHideCommand extends SequentialCommandGroup {
    private final Command lastCommand;

    /**
     * Creates a command that drives to the correct position on the backdrop, places the yellow pixel on the (hopefully)
     *  * correct position, and drives into the corner.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param elbowSubsystem Reference to the elbowSubsystem
     * @param linearSlideSubsystem Reference to the linearSlideSubsystem
     * @param boxSubsystem Reference to the boxReleaseSubsystem
     * @param autoPosition The starting auto position
     */
    public AutoPlaceYellowAndHideCommand(DriveSubsystem driveSubsystem, ElbowSubsystem elbowSubsystem, LinearSlideSubsystem linearSlideSubsystem, BoxSubsystem boxSubsystem, AutoPosition autoPosition) {
        lastCommand = new WaitCommand(1);
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE)
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90)),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxSubsystem),
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(-90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -15),
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-4)),
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(0))
            );
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE)
            addCommands(
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxSubsystem),
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(-90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -24),
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-4)),
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(0))
            );
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE)
            addCommands(
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(8)),
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(90)),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxSubsystem),
                    new MoveCommand(driveSubsystem, MovementType.TURN, autoPosition.flip(-90)),
                    new MoveCommand(driveSubsystem, MovementType.DRIVE, -30),
                    new MoveCommand(driveSubsystem, MovementType.STRAFE, autoPosition.flip(-8)),
                    new MoveCommand(driveSubsystem, MovementType.TURN_TO_HEADING, autoPosition.flip(0))
            );
        addCommands(lastCommand);
        addRequirements(driveSubsystem, elbowSubsystem, linearSlideSubsystem, boxSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
