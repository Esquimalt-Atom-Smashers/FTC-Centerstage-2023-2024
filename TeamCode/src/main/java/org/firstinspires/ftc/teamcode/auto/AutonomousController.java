package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.CommandManager;

public class AutonomousController {
    enum AutonomousState {
        MOVING_TO_SPIKE_MARKS,
        PLACING_PURPLE,
        MOVING_TO_BACKDROP,
        MOVING_TO_PLACE_YELLOW,
        IDLE
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Robot robot;
    private final CommandManager commandManager;

    private Command currentCommand;
    private AutonomousState state;

    private final AutoPosition autoPosition;

    public AutonomousController(LinearOpMode opMode, boolean isBlueAlliance, boolean isUpstage, boolean isPlacingYellow) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        robot = new Robot(opMode, true, true);
        commandManager = new CommandManager(robot);
        autoPosition = new AutoPosition(isBlueAlliance, isPlacingYellow, isUpstage);

        if (isBlueAlliance) robot.getLedSubsystem().setBlue();
        else robot.getLedSubsystem().setRed();
    }

    public void start() {
        state = AutonomousState.MOVING_TO_SPIKE_MARKS;
        scheduleCommand(commandManager.getAutoSetupCommand());
    }

    public void run() {
        switch (state) {
            case MOVING_TO_SPIKE_MARKS:
                if (canContinue()) {
                    if (robot.getDistanceSensorSubsystem().isLeftBlocked())
                        autoPosition.setSpikeMark(autoPosition.isBlue ? AutoPosition.SpikeMark.UPSTAGE : AutoPosition.SpikeMark.DOWNSTAGE);
                    else if (robot.getDistanceSensorSubsystem().isRightBlocked())
                        autoPosition.setSpikeMark(autoPosition.isBlue ? AutoPosition.SpikeMark.DOWNSTAGE : AutoPosition.SpikeMark.UPSTAGE);
                    else
                        autoPosition.setSpikeMark(AutoPosition.SpikeMark.MIDDLE);
                    state = AutonomousState.PLACING_PURPLE;
                    scheduleCommand(commandManager.getAutoDriveAndPlacePurpleCommand(autoPosition));
                }
                break;
            case PLACING_PURPLE:
                if (canContinue()) {
                    state = autoPosition.isPlacingYellow ? AutonomousState.MOVING_TO_BACKDROP : AutonomousState.IDLE;
                    scheduleCommand(commandManager.getAutoDriveFromPurpleCommand(autoPosition));
                }
                break;
            case MOVING_TO_BACKDROP:
                if (canContinue()) {
                    scheduleCommand(commandManager.getAutoPlaceYellowAndHideCommand(autoPosition));
                    state = AutonomousState.MOVING_TO_PLACE_YELLOW;
                }
                break;
            case MOVING_TO_PLACE_YELLOW:
                if (canContinue()) {
                    state = AutonomousState.IDLE;
                }
                break;
            case IDLE:
                break;

        }
        CommandScheduler.getInstance().run();
    }

    public AutoPosition.SpikeMark getSpikeMark() {
        return autoPosition.spikeMark;
    }

    public CommandManager getCommandManager() {
        return commandManager;
    }

    private boolean canContinue() {
        return currentCommand.isFinished();
    }

    private void scheduleCommand(Command command) {
        currentCommand = command;
        command.schedule();
    }
}
