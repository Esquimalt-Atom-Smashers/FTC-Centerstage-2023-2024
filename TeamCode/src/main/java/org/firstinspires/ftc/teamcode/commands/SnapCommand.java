package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command that takes over turning from the driver to rotate the robot to a
 * heading while the driver drives.
 */
public class SnapCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GamepadEx driver;
    private final double heading;

    /**
     * Creates a command that rotates the robot to a specified angle while the driver is driving.
     *
     * @param driveSubsystem Reference to the driveSubsystem
     * @param gamepad Reference to the driver gamepad
     * @param targetHeading The target heading in degrees
     */
    public SnapCommand(DriveSubsystem driveSubsystem, GamepadEx gamepad, double targetHeading) {
        this.driveSubsystem = driveSubsystem;
        this.driver = gamepad;
        this.heading = targetHeading;
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
