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
        MOVING_TO_PLACE_YELLOW,
        PLACING_YELLOW,
        HIDING,
        IDLE
    }

    public enum SpikeMark {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Robot robot;
    private final CommandManager commandManager;

    private WaitCommand currentCommand;
    private AutonomousState state;
    private SpikeMark spikeMarkPosition;

    private final boolean isBlueAlliance;
    private final boolean isUpstage;
    private final boolean isPlacingYellow;

    public NewAutonomousController(LinearOpMode opMode, boolean isBlueAlliance, boolean isUpstage, boolean isPlacingYellow) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        robot = new Robot(opMode, true, true);
        commandManager = new CommandManager(robot);

        this.isBlueAlliance = isBlueAlliance;
        this.isUpstage = isUpstage;
        this.isPlacingYellow = isPlacingYellow;
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
                        spikeMarkPosition = SpikeMark.LEFT;
                    else if (robot.getDistanceSensorSubsystem().isRightBlocked())
                        spikeMarkPosition = SpikeMark.RIGHT;
                    else
                        spikeMarkPosition = SpikeMark.MIDDLE;
                    state = AutonomousState.PLACING_PURPLE;
                    scheduleCommand(commandManager.getAutoDriveAndPlacePurpleCommand(spikeMarkPosition));
                }
                break;
            case PLACING_PURPLE:
                if (canContinue()) {
                    if (isPlacingYellow) {
                        scheduleCommand(commandManager.getAutoPlaceYellowAndHideCommand(spikeMarkPosition));
                        state = AutonomousState.MOVING_TO_PLACE_YELLOW;
                    }
                    else {
                        // TODO: Make it turn the right direction at the end no matter what
                        state = AutonomousState.IDLE;
                    }
                }
                break;
            case MOVING_TO_PLACE_YELLOW:
                if (canContinue()) {
                    state = AutonomousState.PLACING_YELLOW;
                }
                break;
            case PLACING_YELLOW:
                if (canContinue()) {
                    state = AutonomousState.HIDING;
                }
                break;
            case HIDING:
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
        return spikeMarkPosition;
    }

    public CommandManager getCommandManager() {
        return commandManager;
    }

    private boolean canContinue() {
        return currentCommand.isFinished();
    }

    private void scheduleCommand(Command command, double waitTime) {
        SequentialCommandGroup group = new SequentialCommandGroup(command);
        currentCommand = new WaitCommand((long) waitTime);
        group.addCommands(currentCommand);
        group.schedule();
    }

    private void scheduleCommand(Command command) {
        scheduleCommand(command, 1);
    }
}
