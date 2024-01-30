package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.auto.AutoPosition;
import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoDriveFromPurpleCommand extends SequentialCommandGroup {
    public AutoDriveFromPurpleCommand(DriveSubsystem driveSubsystem, AutoPosition autoPosition) {
        int multiplier = autoPosition.isBlue ? 1 : -1;
        if (!autoPosition.isPlacingYellow) {
            addCommands(
                    new TurnToHeadingCommand(driveSubsystem, 0),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 0, Constants.DriveConstants.AUTO_TURN_SPEED/2)
            );
            return;
        }
        if (autoPosition.spikeMark == AutoPosition.SpikeMark.UPSTAGE) {
            addCommands(
                    new TurnCommand(driveSubsystem, 90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 25),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, 8 * multiplier)
            );
        }
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.MIDDLE) {
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
        else if (autoPosition.spikeMark == AutoPosition.SpikeMark.DOWNSTAGE) {
            addCommands(
                    new DriveCommand(driveSubsystem, -4),
                    new WaitCommand(250),
                    new TurnCommand(driveSubsystem, 180 * multiplier, 0.3),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 26)
            );
        }
    }
}