package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double target;

    public AutoDriveCommand(DriveSubsystem subsystem, double target) {
        driveSubsystem = subsystem;
        this.target = target;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.setForwardTarget(target);
    }

    @Override
    public void execute() {
        driveSubsystem.forwardPID();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.isAtForwardTarget();
    }
}
