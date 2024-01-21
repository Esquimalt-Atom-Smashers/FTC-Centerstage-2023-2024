package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnCommand extends SequentialCommandGroup {
    private Command lastCommand;


    public TurnCommand(DriveSubsystem driveSubsystem, double angle) {
        double targetHeading = driveSubsystem.getHeading() + angle;
        lastCommand = new WaitCommand(1);

        addCommands(
                // Turn by angle to the target
                new TurnByAngleCommand(driveSubsystem, angle, Constants.DriveConstants.AUTO_TURN_SPEED),
                new WaitCommand(250),
                // Then correct at a slower pace using the target heading
                new TurnToHeadingCommand(driveSubsystem, targetHeading, Constants.DriveConstants.AUTO_TURN_SPEED / 2),
                lastCommand
            );

        addRequirements(driveSubsystem);
    }



    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
