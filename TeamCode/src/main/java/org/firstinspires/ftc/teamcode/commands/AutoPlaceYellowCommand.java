package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BoxReleaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

/**
 * Command that moves the arm to place the yellow pixel on the backdrop.
 */
public class AutoPlaceYellowCommand extends SequentialCommandGroup {
    private final Command lastCommand;

    /**
     * Creates a command that places the yellow pixel on the backdrop.
     *
     * @param elbowSubsystem Reference to the elbowSubsystem
     * @param linearSlideSubsystem Reference to the linearSlideSubsystem
     * @param boxReleaseSubsystem Reference to the boxReleaseSubsystem
     */
    public AutoPlaceYellowCommand(ElbowSubsystem elbowSubsystem, LinearSlideSubsystem linearSlideSubsystem, BoxReleaseSubsystem boxReleaseSubsystem) {
        lastCommand = new WaitCommand(1);
        addCommands(
                // TODO: Make this happen somewhere else (maybe while placing the purple pixel?)
                // First move the slide and arm in to make sure their zeroes are good
                new MoveSlideCommand(linearSlideSubsystem, linearSlideSubsystem.getInPosition()),
                new MoveElbowCommand(elbowSubsystem, 0),
                // Move the arm to low scoring position
                new MoveElbowCommand(elbowSubsystem, elbowSubsystem.getLowScoringPosition()),
                new MoveSlideCommand(linearSlideSubsystem, linearSlideSubsystem.getLowScoringPosition()),
                // Open then close the box
                new InstantCommand(boxReleaseSubsystem::openBox, boxReleaseSubsystem),
                new WaitCommand(500),
                new InstantCommand(boxReleaseSubsystem::closeBox, boxReleaseSubsystem),
                // Move the arm back down
                // TODO: Make this happen somewhere else (maybe while hiding)
                new MoveSlideCommand(linearSlideSubsystem, linearSlideSubsystem.getInPosition()),
                new MoveElbowCommand(elbowSubsystem, elbowSubsystem.getLevelPosition()),
                lastCommand
        );
        addRequirements(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
