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

public class NewAutonomousController {
    enum AutonomousState {
        MOVING_TO_SPIKE_MARKS,
        PLACING_PURPLE,
        MOVING_TO_BACKDROP,
        MOVING_TO_PLACE_YELLOW,
        IDLE
    }

    public enum SpikeMark {
        UPSTAGE,
        MIDDLE,
        DOWNSTAGE
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Robot robot;
    private final CommandManager commandManager;

    private WaitCommand currentCommand;
    private AutonomousState state;

    private final AutoPosition autoPosition;

    public NewAutonomousController(LinearOpMode opMode, boolean isBlueAlliance, boolean isUpstage, boolean isPlacingYellow) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        robot = new Robot(opMode, true, true);
        commandManager = new CommandManager(robot);

        autoPosition = new AutoPosition(isBlueAlliance, isPlacingYellow, isUpstage);
    }

    public void start() {
        state = AutonomousState.MOVING_TO_SPIKE_MARKS;
        scheduleCommand(commandManager.getAutoSetupCommand(), 1000);
    }

    public void run() {
        switch (state) {
            case MOVING_TO_SPIKE_MARKS:
                if (canContinue()) {
                    if (robot.getDistanceSensorSubsystem().isLeftBlocked())
                        autoPosition.setSpikeMark(autoPosition.isBlue ? SpikeMark.UPSTAGE : SpikeMark.DOWNSTAGE);
                    else if (robot.getDistanceSensorSubsystem().isRightBlocked())
                        autoPosition.setSpikeMark(autoPosition.isBlue ? SpikeMark.DOWNSTAGE : SpikeMark.UPSTAGE);
                    else
                        autoPosition.setSpikeMark(SpikeMark.MIDDLE);
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

    public SpikeMark getSpikeMark() {
        return autoPosition.spikeMark;
    }

    public CommandManager getCommandManager() {
        return commandManager;
    }

    private boolean canContinue() {
        return currentCommand.isFinished();
    }

    private void scheduleCommand(Command command, int waitTime) {
        SequentialCommandGroup group = new SequentialCommandGroup(command);
        currentCommand = new WaitCommand(waitTime);
        group.addCommands(currentCommand);
        group.schedule();
    }

    private void scheduleCommand(Command command) {
        scheduleCommand(command, 1);
    }
}
