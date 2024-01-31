package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command that uses the driveSubsystem to turn by an angle, waits a bit, then corrects the oversteering by turning to the target heading.
 */
public class TurnCommand extends SequentialCommandGroup {
    private final Command lastCommand;

    /**
     * Creates a command that turns a specified angle, waits a bit, then turns to the target heading to correct. Uses a specified
     * speed.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param angle The angle to turn by in degrees
     * @param speed The speed to turn at
     */
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

    /**
     * Creates a command that turns a specified angles, waits a bit, then turns to the target heading to correct. Uses
     * default autonomous turn speed.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param angle The angle to turn by in degrees
\     */
    public TurnCommand(DriveSubsystem driveSubsystem, double angle) {
        this(driveSubsystem, angle, Constants.DriveConstants.AUTO_TURN_SPEED);
    }



    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
