package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TurnByAngleCommand extends CommandBase {
    protected final DriveSubsystem driveSubsystem;
    protected final double speed;
    protected double angle;

    public TurnByAngleCommand(DriveSubsystem driveSubsystem, double angle, double speed) {
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.speed = speed;
        addRequirements(driveSubsystem);
    }

    public TurnByAngleCommand(DriveSubsystem driveSubsystem, double angle) {
        this(driveSubsystem, angle, Constants.DriveConstants.AUTO_TURN_SPEED);
    }

    @Override
    public void initialize() {
        if (angle == 0) {
            cancel();
            return;
        }
        driveSubsystem.turnAsync(angle, speed);
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
        return driveSubsystem.isFinishedTurning();
    }
}
