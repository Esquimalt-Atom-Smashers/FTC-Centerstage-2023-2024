package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.BoxReleaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class AutoPlaceYellowAndHideCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoPlaceYellowAndHideCommand(DriveSubsystem driveSubsystem, ElbowSubsystem elbowSubsystem, LinearSlideSubsystem linearSlideSubsystem, BoxReleaseSubsystem boxReleaseSubsystem, AutoPosition autoPosition) {
        int multiplier = autoPosition.isBlue ? 1 : -1;
        lastCommand = new WaitCommand(1);
        if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.UPSTAGE)
            addCommands(
                    new TurnToHeadingCommand(driveSubsystem, 90 * multiplier),
                    new WaitCommand(250),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem),
                    new TurnCommand(driveSubsystem, -90 * multiplier, 0.3),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -15),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, -4 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0 * multiplier)
            );
        else if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.MIDDLE)
            addCommands(
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem),
                    new TurnCommand(driveSubsystem, -90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -24),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, -4 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0 * multiplier)
            );
        else if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.DOWNSTAGE)
            addCommands(
                    new StrafeCommand(driveSubsystem, 8 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 90 * multiplier),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem),
                    new TurnCommand(driveSubsystem, -90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -30),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, -8 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0 * multiplier)
            );
        addCommands(lastCommand);
        addRequirements(driveSubsystem, elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
