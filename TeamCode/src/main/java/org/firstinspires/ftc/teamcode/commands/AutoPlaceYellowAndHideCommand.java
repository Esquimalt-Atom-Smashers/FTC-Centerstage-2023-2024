package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

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
                    new TurnToHeadingCommand(driveSubsystem, autoPosition.flipMovement(90)),
                    new WaitCommand(250),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxSubsystem),
                    new TurnCommand(driveSubsystem, autoPosition.flipMovement(-90), 0.3),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -15),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, autoPosition.flipMovement(-4)),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, autoPosition.flipMovement(0))
            );
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE)
            addCommands(
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxSubsystem),
                    new TurnCommand(driveSubsystem, autoPosition.flipMovement(-90)),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -24),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, autoPosition.flipMovement(-4)),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, autoPosition.flipMovement(0))
            );
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE)
            addCommands(
                    new StrafeCommand(driveSubsystem, autoPosition.flipMovement(8)),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, autoPosition.flipMovement(90)),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxSubsystem),
                    new TurnCommand(driveSubsystem, autoPosition.flipMovement(-90)),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -30),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, autoPosition.flipMovement(-8)),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, autoPosition.flipMovement(0))
            );
        addCommands(lastCommand);
        addRequirements(driveSubsystem, elbowSubsystem, linearSlideSubsystem, boxSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
