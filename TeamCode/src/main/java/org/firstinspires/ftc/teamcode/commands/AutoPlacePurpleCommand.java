package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class AutoPlacePurpleCommand extends SequentialCommandGroup {
    private final Command lastCommand;
    public AutoPlacePurpleCommand(IntakeSubsystem intakeSubsystem) {
        lastCommand = new WaitCommand(1);
        addCommands(
                new InstantCommand(intakeSubsystem::downPosition, intakeSubsystem),
                new WaitCommand(250),
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
