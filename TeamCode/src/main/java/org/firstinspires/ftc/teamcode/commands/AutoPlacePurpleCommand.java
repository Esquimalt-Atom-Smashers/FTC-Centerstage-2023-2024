package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command that uses the intakeSubsystem to place the purple pixel during autonomous.
 */
public class AutoPlacePurpleCommand extends SequentialCommandGroup {
    private final Command lastCommand;

    /**
     * Creates the command that places the purple pixel during autonomous.
     *
     * @param intakeSubsystem Reference to the intakeSubsystem
     */
    public AutoPlacePurpleCommand(IntakeSubsystem intakeSubsystem) {
        lastCommand = new WaitCommand(1);
        addCommands(
                new InstantCommand(intakeSubsystem::downPosition, intakeSubsystem),
                new WaitCommand(500),
                new InstantCommand(() -> intakeSubsystem.intake(0.5), intakeSubsystem),
                new WaitCommand(500),
                new InstantCommand(intakeSubsystem::stopMotor, intakeSubsystem),
                new InstantCommand(intakeSubsystem::upPosition, intakeSubsystem),
                lastCommand
        );

        addRequirements(intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
