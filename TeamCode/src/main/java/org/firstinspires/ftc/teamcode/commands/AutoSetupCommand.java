package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command that drives the robot forwards to sense for the team prop.
 */
public class AutoSetupCommand extends SequentialCommandGroup {
    private final Command lastCommand;

    /**
     * Creates a command that drives forwards to sense for the team prop.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param intakeSubsystem Reference to the intakeSubsystem
     */
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
