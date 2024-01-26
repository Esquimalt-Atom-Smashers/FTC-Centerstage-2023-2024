package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class AutoDriveAndPlacePurpleCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoDriveAndPlacePurpleCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, AutoPosition autoPosition) {
        int multiplier = autoPosition.isBlue ? 1 : -1;
        // This starts from 30 inches forward
        if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.UPSTAGE) {
            addCommands(
                    // Driving to the correct position
                    new DriveCommand(driveSubsystem, -18),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, -8 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0 * multiplier),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }

        else if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.MIDDLE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -7),
                    new AutoPlacePurpleCommand(intakeSubsystem)
            );
        }
        else if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, -90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, -2),
                    new WaitCommand(250),
                    // Placing purple
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
