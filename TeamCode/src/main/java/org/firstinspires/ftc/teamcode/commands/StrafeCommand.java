package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command that uses the driveSubsystem to strafe right and left.
 */
public class StrafeCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double distance;

    /**
     * Creates a command that strafes the robot specified inches right or left.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param distance The distance in inches
     */
    public StrafeCommand(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (distance == 0) {
            cancel();
            return;
        }
        driveSubsystem.strafeByDistanceAsync(distance);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.isFinishedMoving();
    }
}
