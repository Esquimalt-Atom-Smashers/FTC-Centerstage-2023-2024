package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double distance;

    public DriveCommand(DriveSubsystem driveSubsystem, double distance) {
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
        driveSubsystem.driveByDistanceAsync(distance);
    }

    // We don't need to put anything in here because isFinished should be called
    // each time which will check to see if it's done yet
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