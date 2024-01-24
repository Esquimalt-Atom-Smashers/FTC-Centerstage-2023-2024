package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class AutoSetupCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoSetupCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        lastCommand = new WaitCommand(1);
        addCommands(
                new InstantCommand(intakeSubsystem::upPosition, intakeSubsystem),
                new DriveCommand(driveSubsystem, 32),
                lastCommand
        );
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
