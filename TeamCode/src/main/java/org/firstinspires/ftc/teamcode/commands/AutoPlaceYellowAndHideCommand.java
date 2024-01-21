package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.BoxReleaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class AutoPlaceYellowAndHideCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoPlaceYellowAndHideCommand(DriveSubsystem driveSubsystem, ElbowSubsystem elbowSubsystem, LinearSlideSubsystem linearSlideSubsystem, BoxReleaseSubsystem boxReleaseSubsystem, NewAutonomousController.SpikeMark spikeMarkPosition) {
        lastCommand = new WaitCommand(1);
        if (spikeMarkPosition == NewAutonomousController.SpikeMark.LEFT)
            // TODO: Be very careful testing this
            addCommands(
                    new StrafeCommand(driveSubsystem, -6),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem),
                    new TurnCommand(driveSubsystem, -90),
                    new DriveCommand(driveSubsystem, -20)
            );
        else if (spikeMarkPosition == NewAutonomousController.SpikeMark.MIDDLE)
            // TODO: Be very careful testing this
            addCommands(
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem),
                    new TurnCommand(driveSubsystem, -90),
                    new DriveCommand(driveSubsystem, -26)
            );
        else if (spikeMarkPosition == NewAutonomousController.SpikeMark.RIGHT)
            // TODO: Be very careful testing this
            addCommands(
                    new StrafeCommand(driveSubsystem, 6),
                    new AutoPlaceYellowCommand(elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem),
                    new TurnCommand(driveSubsystem, -90),
                    new DriveCommand(driveSubsystem, -30)
            );
        addCommands(lastCommand);
        addRequirements(driveSubsystem, elbowSubsystem, linearSlideSubsystem, boxReleaseSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
