package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

public class CommandManager {
    private final Command defaultClawCommand;
    private final Command openClawCommand;
    private final Command defaultDriveCommand;
    private final Command droneModeCommand;

    public CommandManager(Robot robot) {
        defaultClawCommand = new RunCommand(() -> {
            if (robot.getOperatorGamepad().getButton(GamepadKeys.Button.LEFT_BUMPER)) robot.getClawSubsystem().closeClaw();
            if (robot.getOperatorGamepad().getButton(GamepadKeys.Button.RIGHT_BUMPER)) robot.getClawSubsystem().openClaw();
        }, robot.getClawSubsystem());

        openClawCommand = new SequentialCommandGroup(
                new InstantCommand(() -> robot.getClawSubsystem().openClaw(), robot.getClawSubsystem()),
                new WaitCommand(250),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), Constants.LinearSlideConstants.IN_POSITION)
        );

        defaultDriveCommand = new RunCommand(() -> robot.getDriveSubsystem().drive(robot.getDriverGamepad().getLeftY(), robot.getDriverGamepad().getLeftX(), robot.getDriverGamepad().getRightX()), robot.getDriveSubsystem());

        droneModeCommand = new SequentialCommandGroup(
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.SHOOTING_DRONE)),
                new MoveElbowCommand(robot.getElbowSubsystem(), Constants.ElbowConstants.DRONE_LAUNCH_POSITION)
        );
    }

    public Command getDefaultClawCommand() {
        return defaultClawCommand;
    }

    public Command getOpenClawCommand() {
        return openClawCommand;
    }

    public Command getDefaultDriveCommand() {
        return defaultDriveCommand;
    }

    public Command getDroneModeCommand() {
        return droneModeCommand;
    }
}
