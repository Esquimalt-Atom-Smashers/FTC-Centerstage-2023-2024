package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.AutonomousController;

public class CommandManager {
    /** Default command for ClawSubsystem */
    private final Command defaultClawCommand;
    /** Command that opens the claw and moves the slide home */
    private final Command openClawCommand;
    /** Default command for DriveSubsystem */
    private final Command defaultDriveCommand;
    /** Command that moves the arm up and enters launching drone mode */
    private final Command droneModeCommand;
    /** Command that launches the drone */
    private final Command droneLaunchCommand;
    /** Command that exits drone launching mode */
    private final Command droneCancelCommand;
    /** Default command for ElbowSubsystem */
    private final Command defaultElbowCommand;
    /** Default command for LinearSlideSubsystem */
    private final Command defaultSlideCommand;
    /** Default command for WinchSubsystem */
    private final Command defaultWinchCommand;
    /** Command that lowers arm and intake and enters intake mode */
    private final Command intakeModeCommand;
    /** Command that exits intake mode */
    private final Command intakeCancelCommand;
    /** Command that picks pixels up and exits intake mode */
    private final Command pickupPixelsCommand;
    /** Command that moves the arm to low scoring position */
    private final Command highScoringPositionCommand;
    /** Command that moves the arm to medium scoring position */
    private final Command mediumScoringPositionCommand;
    /** Command that moves the arm to high scoring position */
    private final Command lowScoringPositionCommand;
    /** Command that moves the arm to home position */
    private final Command homePostionCommand;
    /** Command run at the start of driver controlled */
    private final Command setupCommand;
    /** Command run at the start of autonomous */
    private final Command autoSetupCommand;
    /** Command that moves the intake to place the purple pixel */
    private final Command autoPlacePurpleCommand;
    /** Command that moves the arm to the placing position */
    private final Command autoMoveArmCommand;
    /** Command that places the yellow pixel */
    private final Command autoPlaceYellowCommand;

    public CommandManager(Robot robot) {
        defaultClawCommand = new RunCommand(() -> {
            if (robot.getOperatorGamepad().getButton(GamepadKeys.Button.LEFT_BUMPER)) robot.getClawSubsystem().closeClaw();
            if (robot.getOperatorGamepad().getButton(GamepadKeys.Button.RIGHT_BUMPER)) robot.getClawSubsystem().openClaw();
        }, robot.getClawSubsystem());

        openClawCommand = new SequentialCommandGroup(
                new InstantCommand(() -> robot.getClawSubsystem().openClaw(), robot.getClawSubsystem()),
                new WaitCommand(250),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition())
        );

        defaultDriveCommand = new RunCommand(() -> robot.getDriveSubsystem().drive(robot.getDriverGamepad().getLeftY(), robot.getDriverGamepad().getLeftX(), robot.getDriverGamepad().getRightX()), robot.getDriveSubsystem());

        droneModeCommand = new SequentialCommandGroup(
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.SHOOTING_DRONE)),
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDroneLaunchPosition())
        );

        droneLaunchCommand = new SequentialCommandGroup(
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.DRIVING)),
                new InstantCommand(robot.getDroneSubsystem()::release, robot.getDroneSubsystem())
        );

        droneCancelCommand = new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.DRIVING));

        defaultElbowCommand = new RunCommand(() -> {
            if (robot.getOperatorGamepad().getLeftY() >= 0.1) robot.getElbowSubsystem().raiseManually(1);
            else if (robot.getOperatorGamepad().getLeftY() <= -0.1) robot.getElbowSubsystem().lowerManually(1);
            else robot.getElbowSubsystem().stopMotor();
        }, robot.getElbowSubsystem());

        // TODO: Check this after
        defaultSlideCommand = new RunCommand(() -> {
            if (robot.getOperatorGamepad().getRightY() >= 0.1) robot.getLinearSlideSubsystem().extendManually(1);
            else if (robot.getOperatorGamepad().getRightY() <= -0.1) robot.getLinearSlideSubsystem().retractManually(1);
            else robot.getLinearSlideSubsystem().stopMotor();
        }, robot.getLinearSlideSubsystem());

        defaultWinchCommand = new RunCommand(() -> {
            if (robot.getDriverGamepad().getButton(GamepadKeys.Button.A)) robot.getWinchSubsystem().winch();
            else if (robot.getDriverGamepad().getButton(GamepadKeys.Button.B)) robot.getWinchSubsystem().unwinch();
            else robot.getWinchSubsystem().stopMotor();
        }, robot.getWinchSubsystem());

        intakeModeCommand = new SequentialCommandGroup(
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.INTAKE)),
                new InstantCommand(robot.getIntakeSubsystem()::downPosition, robot.getIntakeSubsystem()),
                new InstantCommand(robot.getClawSubsystem()::closeClaw, robot.getClawSubsystem()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition()),
                // TODO: Can this go straight to intake position? Is that safe to drive on?
                // If so, we can remove the movement in pickup pixels
                // I don't think we can
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDrivingPosition()),
                new InstantCommand(robot.getIntakeSubsystem()::intake, robot.getIntakeSubsystem())
        );

        intakeCancelCommand = new SequentialCommandGroup(
                new InstantCommand(robot.getIntakeSubsystem()::stopMotor, robot.getIntakeSubsystem()),
                new InstantCommand(robot.getIntakeSubsystem()::mediumPosition, robot.getIntakeSubsystem()),
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.DRIVING))
        );

        pickupPixelsCommand = new SequentialCommandGroup(
                // Set the scoring state to loading pixels
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.LOADING_PIXELS)),
                // Move the arm down to pick up the pixels
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getIntakePosition()),
                // TODO: Check if we still need to move the intake up and down
                // Stop the intake and move it up
                new InstantCommand(robot.getIntakeSubsystem()::stopMotor, robot.getIntakeSubsystem()),
                new InstantCommand(robot.getIntakeSubsystem()::mediumPosition, robot.getIntakeSubsystem()),
                // Move the elbow to level position
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getLevelPosition()),
                // Move the intake down and wait 250ms for it to get there
                new InstantCommand(robot.getIntakeSubsystem()::downPosition, robot.getIntakeSubsystem()),
//              // Set the state to driving again
                new InstantCommand(() -> robot.setScoringState(Robot.ScoringState.DRIVING))
        );

        lowScoringPositionCommand = new SequentialCommandGroup(
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getLowScoringPosition()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getLowScoringPosition())
        );

        mediumScoringPositionCommand = new SequentialCommandGroup(
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getMediumScoringPosition()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getMediumScoringPosition())
        );

        highScoringPositionCommand = new SequentialCommandGroup(
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getHighScoringPosition()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getHighScoringPosition())
        );

        homePostionCommand = new SequentialCommandGroup(
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition()),
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDrivingPosition())
        );

        setupCommand = new SequentialCommandGroup(
                new InstantCommand(robot.getDroneSubsystem()::startPosition, robot.getDroneSubsystem())
        );

//        autoSetupCommand = new SequentialCommandGroup(
//                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition()),
//                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getIntakePosition()),
//                new InstantCommand(robot.getClawSubsystem()::closeClawSingle, robot.getClawSubsystem()),
//                new InstantCommand(robot.getIntakeSubsystem()::mediumPosition, robot.getIntakeSubsystem()),
//                new WaitCommand(500),
//                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDrivingPosition())
//        );
        //TODO: Redo command
        autoSetupCommand = new SequentialCommandGroup(
                new InstantCommand(robot.getIntakeSubsystem()::downPosition, robot.getIntakeSubsystem()),
                new InstantCommand(robot.getClawSubsystem()::closeClaw, robot.getClawSubsystem()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition()),
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDrivingPosition())
        );


//        autoPlacePurpleCommand = new SequentialCommandGroup(
//                new InstantCommand(robot.getIntakeSubsystem()::downPosition, robot.getIntakeSubsystem()),
//                new WaitCommand(250),
//                new InstantCommand(robot.getIntakeSubsystem()::intake, robot.getIntakeSubsystem()),
//                new WaitCommand(250),
//                new InstantCommand(robot.getIntakeSubsystem()::stopMotor, robot.getIntakeSubsystem()),
//                new InstantCommand(robot.getIntakeSubsystem()::mediumPosition, robot.getIntakeSubsystem())
//        );
        //TODO: Redo command
        autoPlacePurpleCommand = new SequentialCommandGroup(
                new InstantCommand(robot.getIntakeSubsystem()::outtake, robot.getIntakeSubsystem()),
                new WaitCommand(250),
                new InstantCommand(robot.getIntakeSubsystem()::stopMotor, robot.getIntakeSubsystem())
        );

//        autoMoveArmCommand = new SequentialCommandGroup(
//                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getLowScoringPosition()),
//                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getLowScoringPosition())
//        );
        //TODO: Redo command
        autoMoveArmCommand = new SequentialCommandGroup(
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getLowScoringPosition()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getLowScoringPosition())
        );

//        autoPlaceYellowCommand = new SequentialCommandGroup(
//                new InstantCommand(robot.getClawSubsystem()::openClaw, robot.getClawSubsystem()),
//                new WaitCommand(750),
//                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition())
//        );
        //TODO: Redo command
        autoPlaceYellowCommand = new SequentialCommandGroup(
                new InstantCommand(robot.getClawSubsystem()::openClaw, robot.getClawSubsystem()),
                new WaitCommand(750),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition())
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

    public Command getDroneLaunchCommand() {
        return droneLaunchCommand;
    }

    public Command getDroneCancelCommand() {
        return droneCancelCommand;
    }

    public Command getDefaultElbowCommand() {
        return defaultElbowCommand;
    }

    public Command getDefaultSlideCommand () {
        return defaultSlideCommand;
    }

    public Command getDefaultWinchCommand () {
        return defaultWinchCommand;
    }

    public Command getIntakeModeCommand () {
        return intakeModeCommand;
    }

    public Command getIntakeCancelCommand () {
        return intakeCancelCommand;
    }

    public Command getPickupPixelsCommand () {
        return pickupPixelsCommand;
    }

    public Command getLowScoringPositionCommand () {
        return lowScoringPositionCommand;
    }

    public Command getMediumScoringPositionCommand () {
        return mediumScoringPositionCommand;
    }

    public Command getHighScoringPositionCommand () {
        return highScoringPositionCommand;
    }

    public Command getHomePostionCommand () {
        return homePostionCommand;
    }

    public Command getSetupCommand() {
        return setupCommand;
    }

    public Command getAutoSetupCommand() {
        return autoSetupCommand;
    }

    public Command getAutoPlacePurpleCommand() {
        return autoPlacePurpleCommand;
    }

    public Command getAutoMoveArmCommand() {
        return autoMoveArmCommand;
    }

    public Command getAutoPlaceYellowCommand() {
        return autoPlaceYellowCommand;
    }
}
