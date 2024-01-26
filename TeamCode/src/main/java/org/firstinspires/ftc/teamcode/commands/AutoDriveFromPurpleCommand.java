package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoDriveFromPurpleCommand extends SequentialCommandGroup {
    public AutoDriveFromPurpleCommand(DriveSubsystem driveSubsystem, AutoPosition autoPosition) {
        int multiplier = autoPosition.isBlue ? 1 : -1;
        if (!autoPosition.isPlacingYellow) {
            addCommands(
                    new TurnToHeadingCommand(driveSubsystem, 0)
            );
            return;
        }
        if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.UPSTAGE) {
            addCommands(
                    new TurnCommand(driveSubsystem, 90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 24),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, 8 * multiplier)
            );
        }
        else if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.MIDDLE) {
            addCommands(
                    new TurnCommand(driveSubsystem, 90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 32),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, 6 * multiplier),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 90 * multiplier)
            );
        }
        else if (autoPosition.spikeMark == NewAutonomousController.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, 180 * multiplier, 0.3),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 22)
            );
        }
    }
}