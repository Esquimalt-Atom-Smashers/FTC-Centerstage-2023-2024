package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnCommand extends SequentialCommandGroup {
    private final Command lastCommand;


    public TurnCommand(DriveSubsystem driveSubsystem, double angle, double speed) {
        double targetHeading = driveSubsystem.getHeading() + angle;
        lastCommand = new WaitCommand(1);

        addCommands(
                // Turn by angle to the target
                new TurnByAngleCommand(driveSubsystem, angle, speed),
                new WaitCommand(250),
                // Then correct at a slower pace using the target heading
                new TurnToHeadingCommand(driveSubsystem, targetHeading, speed / 2),
                lastCommand
            );

        addRequirements(driveSubsystem);
    }

    public TurnCommand(DriveSubsystem driveSubsystem, double angle) {
        this(driveSubsystem, angle, Constants.DriveConstants.AUTO_TURN_SPEED);
    }



    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
