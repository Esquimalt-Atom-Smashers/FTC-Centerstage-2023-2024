package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class AutoDriveAndPlacePurpleCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoDriveAndPlacePurpleCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, NewAutonomousController.SpikeMark spikeMarkPosition, boolean placingYellow) {
        // This starts from 30 inches forward
        if (spikeMarkPosition == NewAutonomousController.SpikeMark.LEFT) {
            addCommands(
                    // TODO: Test this chunk
                    // Driving to the correct position
                    new TurnCommand(driveSubsystem, 90),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, 5),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -2),
                    new WaitCommand(250),
                    // Placing the purple pixel
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
            // TODO: Be very careful testing this
            if (placingYellow)
                addCommands(
                    new StrafeCommand(driveSubsystem, 12),
                    new DriveCommand(driveSubsystem, 40.0 / 2), // Based on my math this is right, but 40 is very scary so I'm halving it for now
                    new StrafeCommand(driveSubsystem, -20)
                    // We are done, we should be at the center of the board
                );
        }

        else if (spikeMarkPosition == NewAutonomousController.SpikeMark.MIDDLE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -4.5),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
            if (placingYellow)
                addCommands(
                        // TODO: Be careful when testing these
                        new TurnCommand(driveSubsystem, 90),
                        new WaitCommand(250),
                        // Should maybe be 28
                        new DriveCommand(driveSubsystem, 30),
                        new WaitCommand(250),
                        new StrafeCommand(driveSubsystem, 6),
                        new WaitCommand(250),
                        new TurnToHeadingCommand(driveSubsystem, 90)
                );
        }
        else if (spikeMarkPosition == NewAutonomousController.SpikeMark.RIGHT) {
            addCommands(
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, -90),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -2),
                    new WaitCommand(250),
                    // Placing purple
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
            if (placingYellow)
                addCommands(
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, 180),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 22)
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
