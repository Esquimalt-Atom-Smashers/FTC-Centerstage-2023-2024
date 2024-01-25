package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoDriveFromPurpleCommand extends SequentialCommandGroup {
    public AutoDriveFromPurpleCommand(DriveSubsystem driveSubsystem, NewAutonomousController.SpikeMark spikeMarkPosition, boolean blue, boolean placingYellow) {
        int multiplier = blue ? 1 : -1;
        if (!placingYellow) {
            addCommands(
                    new TurnToHeadingCommand(driveSubsystem, 0)
            );
            return;
        }
        if (spikeMarkPosition == NewAutonomousController.SpikeMark.LEFT) {
            addCommands(
                    new TurnCommand(driveSubsystem, 90),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 24),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, 8)
//
//                    new DriveCommand(driveSubsystem, 36),
//                    new WaitCommand(250),
//                    new StrafeCommand(driveSubsystem, -20),
//                    new WaitCommand(250),
//                    new TurnToHeadingCommand(driveSubsystem, 90),
//                    new WaitCommand(250)
            );
        }
        else if (spikeMarkPosition == NewAutonomousController.SpikeMark.MIDDLE) {
            addCommands(
                    new TurnCommand(driveSubsystem, 90 * multiplier),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 32),
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
                    new TurnCommand(driveSubsystem, 180 * multiplier, 0.3),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 22)
            );
        }
    }
}