package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.*;

/**
 * Command that performs some movement (driving, strafing, turning), then waits.
 */
public class MoveCommand extends SequentialCommandGroup {
    private Command lastCommand;

    public enum MovementType {
        DRIVE,
        STRAFE,
        TURN,
        TURN_TO_HEADING,
        SLOW_TURN_TO_HEADING
    }

    /**
     * Creates a new command that drives/strafes/turns the robot, then waits a specified amount of time.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param movementType What movement we are going to do (drive/strafe/turn)
     * @param value The distance/angle to move
     * @param waitTime The amount of time to wait after the movement (in milliseconds)
     */
    public MoveCommand(DriveSubsystem driveSubsystem, MovementType movementType, double value, long waitTime) {
        switch (movementType) {
            case DRIVE:
                lastCommand = new DriveCommand(driveSubsystem, value);
                addCommands(lastCommand);
                break;
            case STRAFE:
                lastCommand = new StrafeCommand(driveSubsystem, value);
                addCommands(lastCommand);
                break;
            case TURN:
                lastCommand = new TurnCommand(driveSubsystem, value);
                addCommands(lastCommand);
                break;
            case TURN_TO_HEADING:
                lastCommand = new TurnToHeadingCommand(driveSubsystem, value);
                addCommands(lastCommand);
                break;
            case SLOW_TURN_TO_HEADING:
                lastCommand = new TurnToHeadingCommand(driveSubsystem, value, Constants.DriveConstants.AUTO_TURN_SPEED / 2);
                addCommands(lastCommand);
                break;
        }
        if (waitTime > 0) {
            lastCommand = new WaitCommand(waitTime);
            addCommands(lastCommand);
        }
    }

    /**
     * Creates a new command that drives/strafes/turns the robot, then waits a quarter of a second.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param movementType What movement we are going to do (drive/strafe/turn)
     * @param value The distance/angle to move
     */
    public MoveCommand(DriveSubsystem driveSubsystem, MovementType movementType, double value) {
        this(driveSubsystem, movementType, value, DEFAULT_AUTO_WAIT);
    }

    @Override
    public boolean isFinished() {
        return lastCommand.isFinished();
    }
}
