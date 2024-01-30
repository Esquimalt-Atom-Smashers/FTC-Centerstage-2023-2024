package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class SnapCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GamepadEx driver;
    private final double heading;

    public SnapCommand(DriveSubsystem driveSubsystem, GamepadEx gamepad, double heading) {
        this.driveSubsystem = driveSubsystem;
        this.driver = gamepad;
        this.heading = heading;
    }

    @Override
    public void initialize() {
        driveSubsystem.turnAsync(heading - driveSubsystem.getHeading(), Constants.DriveConstants.AUTO_TURN_SPEED);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(driver.getLeftY(), driver.getLeftX(), driveSubsystem.getAutoTurnSpeed(), true, false, 1);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.isFinishedTurning();
    }
}
