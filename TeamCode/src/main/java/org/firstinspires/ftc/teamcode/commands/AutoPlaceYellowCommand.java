package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BoxReleaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class AutoPlaceYellowCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoPlaceYellowCommand(ElbowSubsystem elbowSubsystem, LinearSlideSubsystem linearSlideSubsystem, BoxReleaseSubsystem boxReleaseSubsystem) {
        lastCommand = new WaitCommand(1);
        addCommands(
                // TODO: Make this happen somewhere else (maybe while placing the purple pixel?
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
                new MoveElbowCommand(elbowSubsystem, elbowSubsystem.getDrivingPosition()),
                lastCommand
        );
        addRequirements(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}