package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoDriveToBackdropCommand extends SequentialCommandGroup {
    public AutoDriveToBackdropCommand(NewAutonomousController.SpikeMark spikeMarkPosition, DriveSubsystem driveSubsystem) {
        if (spikeMarkPosition == NewAutonomousController.SpikeMark.LEFT) {
            addCommands(
                    new StrafeCommand(driveSubsystem, 12),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 36),
                    new WaitCommand(250),
                    new StrafeCommand(driveSubsystem, -20),
                    new WaitCommand(250),
                    new TurnToHeadingCommand(driveSubsystem, 90),
                    new WaitCommand(250)
            );
        }
        else if (spikeMarkPosition == NewAutonomousController.SpikeMark.MIDDLE) {
            addCommands(
                    new TurnCommand(driveSubsystem, 90),
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
                    new TurnCommand(driveSubsystem, 180, 0.3),
                    new WaitCommand(250),
                    new DriveCommand(driveSubsystem, 22)
            );
        }
    }
}