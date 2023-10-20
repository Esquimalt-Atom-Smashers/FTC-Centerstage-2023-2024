package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Instructions.InstructionExecutor;
import org.firstinspires.ftc.teamcode.commands.MoveElbowCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class Robot {
    // OpMode
    private final OpMode opMode;

    // Gamepads
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    //Define subsystems here.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElbowSubsystem elbowSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final LinearSlideSubsystem linearSlideSubsystem;
    private final CameraSubsystem cameraSubsystem;
    // TODO: Make this final again, was only removed because the drone was taken off of the robot
    private DroneSubsystem droneSubsystem;
    private final WinchSubsystem winchSubsystem;

    private final InstructionExecutor instructionExecutor;

    private boolean usingPIDControllers;

    enum DriveState {
        DRIVER_CONTROLLED,
        SNAPPING,
        DETECTING_TAG,
        CENTERING_TAG,
        STEPPING_LEFT,
        STEPPING_RIGHT
    }

    // TODO: Simplify this (maybe use ftclib commands?)
    enum ScoringState {
        STARTING,
        INTAKE,
        LOWERING_CLAW,
        WAITING,
        RAISING_CLAW,
        START_ALIGN,
        LOWER_INTAKE,
        EXTEND,
        END_ALIGN,
        LEVEL,
        LOADED_DRIVING,
        SCORING_POSITION,
        DRIVING,
        MANUAL
    }

    ElapsedTime timer;

    private DriveState driveState = DriveState.DRIVER_CONTROLLED;
    private ScoringState scoringState = ScoringState.STARTING;

    public Robot(OpMode opMode, boolean usingPID) {
        this.opMode = opMode;

        this.usingPIDControllers = usingPID;
        if (!usingPID) scoringState = ScoringState.MANUAL;

        // Initialize the gamepads
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        // Initialize the subsystems
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap);
        clawSubsystem = new ClawSubsystem(opMode.hardwareMap);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap, this);
        cameraSubsystem = new CameraSubsystem(opMode.hardwareMap, opMode.telemetry);
        // TODO: Uncomment when the drone subsystem is back on the robot
//        droneSubsystem = new DroneSubsystem(opMode.hardwareMap);
        winchSubsystem = new WinchSubsystem(opMode.hardwareMap);


        instructionExecutor = new InstructionExecutor();


        // TODO: How do you do instant commands?
        // Controls for individual subsystems:
        // CameraSubsystem

        // ClawSubsystem

        // DriveSubsystem
//        driveSubsystem.setDefaultCommand(new InstantCommand(() -> {
//            driveSubsystem.drive(-driverGamepad.getLeftY(), -driverGamepad.getLeftX(), -driverGamepad.getRightX());
//        }, driveSubsystem));

        // DroneSubsystem

        // ElbowSubsystem
//        elbowSubsystem.setDefaultCommand(new InstantCommand(() -> {
//            if (operatorGamepad.getLeftY() <= -0.1) elbowSubsystem.lowerManually();
//            else if (operatorGamepad.getLeftY() >= 0.1) elbowSubsystem.raiseManually();
//            else elbowSubsystem.stop();
//        }, elbowSubsystem));

        // IntakeSubsystem

        // LinearSlideSubsystem
//        linearSlideSubsystem.setDefaultCommand(new InstantCommand(() -> {
//            if (operatorGamepad.getRightY() >= 0.1) linearSlideSubsystem.retractManually();
//            else if (operatorGamepad.getRightY() <= -0.1) linearSlideSubsystem.extendManually();
//            else linearSlideSubsystem.stop();
//        }, linearSlideSubsystem));

        // PixelSubsystem

        // WinchSubsystem
//        winchSubsystem.setDefaultCommand(new InstantCommand(winchSubsystem::stop));

        // Controls for command groups
        // TODO: Make custom triggers so that you can only move to set positions after we have picked up pixels
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.LOW_SCORING_POSITION, opMode.telemetry),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.LOW_SCORING_POSITION, opMode.telemetry)
        ));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new SequentialCommandGroup(
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.MEDIUM_SCORING_POSITION, opMode.telemetry),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.MEDIUM_SCORING_POSITION, opMode.telemetry)
        ));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new SequentialCommandGroup(
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.HIGH_SCORING_POSITION, opMode.telemetry),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.HIGH_SCORING_POSITION, opMode.telemetry)
        ));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new SequentialCommandGroup(
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.IN_POSITION, opMode.telemetry),
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.INTAKE_POSITION, opMode.telemetry)
        ));


    }

    public Robot(OpMode opMode) {
        this(opMode, true);
    }

    // Perform actions that happen when the robot starts
    public void start() {
//        linearSlideSubsystem.retract();
//        droneSubsystem.startPosition();
    }

    // Main robot control
    public void run() {
        // Control all of the subsystems

        // Drive subsystem controls (driver):
        // Left joystick up and down moves the robot forward and back
        // Left joystick left and right moves the robot left and right
        // Right joystick left and right turns the robot left and right
//        driveLoop();

        CommandScheduler.getInstance().run();


        // FOR DRIVERS:
        // Press A to enter intake mode
        // While in intake mode press X to pick up
        // Press B to enter driving mode

//        if (driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) droneSubsystem.release();

//        if (operatorGamepad.getButton(GamepadKeys.Button.A)) scoringState = ScoringState.INTAKE;
//        if (operatorGamepad.getButton(GamepadKeys.Button.B)) {
//            scoringState = ScoringState.LOADED_DRIVING;
//            intakeSubsystem.stop();
//            intakeSubsystem.downPosition();
//            intakeSubsystem.mediumPosition();
//            elbowSubsystem.levelPosition();
//        }

//        if (operatorGamepad.getLeftY() <= -0.1) elbowSubsystem.lowerManually();
//        else if (operatorGamepad.getLeftY() >= 0.1) elbowSubsystem.raiseManually();
//        else elbowSubsystem.stop();
//
//        if (operatorGamepad.getRightY() >= 0.1) linearSlideSubsystem.retractManually();
//        else if (operatorGamepad.getRightY() <= -0.1) linearSlideSubsystem.extendManually();
//        else linearSlideSubsystem.stop();
//
//        if (operatorGamepad.getButton(GamepadKeys.Button.Y)) clawSubsystem.openClaw();

//        scoringLoop();
//
//        cameraSubsystem.detect();
//        opMode.telemetry.addData("Driving State", driveState);
//        opMode.telemetry.addData("Scoring State", scoringState);
//        elbowSubsystem.printPosition(opMode.telemetry);
//        linearSlideSubsystem.printData(opMode.telemetry);
        opMode.telemetry.update();

        // Instruction controls (driver):
        // Left trigger adds a left align
        // Right trigger adds a right align
        // Both triggers add a center align
        // Left bumper adds a step left
        // Right bumper adds a step right
        // A enters the instructions
        // B cancels the instructions
//        if (isPressed(driverGamepad.left_trigger) && isPressed(driverGamepad.right_trigger))
//            instructionExecutor.addInstruction(this::alignCenter);
//        else if (isPressed(driverGamepad.left_trigger))
//            instructionExecutor.addInstruction(this::alignLeft);
//        else if (isPressed(driverGamepad.right_trigger))
//            instructionExecutor.addInstruction(this::alignRight);
//        if (driverGamepad.left_bumper) instructionExecutor.addInstruction(this::stepLeft);
//        if (driverGamepad.right_bumper) instructionExecutor.addInstruction(this::stepRight);
//        if (driverGamepad.a) instructionExecutor.executeInstructions();
//        if (driverGamepad.b) instructionExecutor.clearInstructions();
    }

    private void driveLoop() {
        switch (driveState) {
            case DRIVER_CONTROLLED:
                driveSubsystem.drive(-driverGamepad.getLeftY(), -driverGamepad.getLeftX(), -driverGamepad.getRightX());
                if (driverGamepad.getButton(GamepadKeys.Button.A)) {
//                    driveSubsystem.autoSnap();
//                    driveState = DriveState.SNAPPING;
                }
                break;
            case SNAPPING:
                if (driveSubsystem.isFinishedSnapping()) {
                    driveState = DriveState.DETECTING_TAG;
                }
                break;
            case DETECTING_TAG:
                break;
            case CENTERING_TAG:
                if (driveSubsystem.isCentered()) {
                    instructionExecutor.getNextInstruction();
                }
                break;
            case STEPPING_LEFT:
                if (driveSubsystem.isFinishedSteppingLeft()) {
                    driveState = DriveState.DRIVER_CONTROLLED;
                }
                break;
            case STEPPING_RIGHT:
                if (driveSubsystem.isFinishedSteppingRight()) {
                    driveState = DriveState.DRIVER_CONTROLLED;
                }
                break;
            default:
                driveState = DriveState.DRIVER_CONTROLLED;
        }
    }

    public void scoringLoop() {
        switch (scoringState) {
            case STARTING:
                intakeSubsystem.mediumPosition();
                clawSubsystem.openClaw();
//                linearSlideSubsystem.retract();
                intakeSubsystem.stop();
                runPIDControllers();
                break;
            case INTAKE:
                intakeSubsystem.downPosition();
                intakeSubsystem.intake();
                clawSubsystem.openClaw();
                linearSlideSubsystem.retract();
                elbowSubsystem.drivingPosition();
                runPIDControllers();
                if (operatorGamepad.getButton(GamepadKeys.Button.X)) {
                    scoringState = ScoringState.LOWERING_CLAW;
                    elbowSubsystem.intakePosition();
                }
                break;
            case LOWERING_CLAW:
                runPIDControllers();
                if (elbowSubsystem.isAtTarget()) {
                    scoringState = ScoringState.WAITING;
                    elbowSubsystem.stop();
                    clawSubsystem.closeClaw();
                    timer = new ElapsedTime();
                }
                break;
            case WAITING:
                if (timer.milliseconds() >= 500) {
                    scoringState = ScoringState.RAISING_CLAW;
                    clawSubsystem.closeClaw();
                    intakeSubsystem.stop();
                    intakeSubsystem.mediumPosition();
                    elbowSubsystem.levelPosition();
                }
                break;
            case RAISING_CLAW:
                runPIDControllers();
                if (elbowSubsystem.isAtTarget()) {
                    scoringState = ScoringState.START_ALIGN;
                    intakeSubsystem.mediumPosition();
                    intakeSubsystem.stop();
                    elbowSubsystem.levelPosition();
//                    linearSlideSubsystem.tiltPosition();
                }
                break;
            case START_ALIGN:
                runPIDControllers();
                if (elbowSubsystem.isAtTarget()) {
                    scoringState = ScoringState.LOWER_INTAKE;
                    intakeSubsystem.downPosition();
                    timer = new ElapsedTime();
//                    elbowSubsystem.levelPosition();
                }
                break;
            case LOWER_INTAKE:
                runPIDControllers();
                if (timer.milliseconds() >= 250) {
                    scoringState = ScoringState.EXTEND;
                    linearSlideSubsystem.tiltPosition();
                }
                break;
            case EXTEND:
                runPIDControllers();
                if (linearSlideSubsystem.isAtTarget()) {
                    scoringState = ScoringState.END_ALIGN;
                    linearSlideSubsystem.tiltPosition();
                }
                break;
            case END_ALIGN:
                runPIDControllers();
                if (linearSlideSubsystem.isAtTarget()) {
                    scoringState = ScoringState.LEVEL;
                    elbowSubsystem.tiltPosition();
//                    linearSlideSubsystem.retract();
                }
                break;
            case LEVEL:
                runPIDControllers();
                if (elbowSubsystem.isAtTarget()) {
                    scoringState = ScoringState.LOADED_DRIVING;
                    elbowSubsystem.levelPosition();
                }
                break;
            // TODO: Change this to move the elbow first, then the slide going up, and vice versa going down
            case LOADED_DRIVING:
                runPIDControllers();
                if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                    scoringState = ScoringState.SCORING_POSITION;
                    elbowSubsystem.lowScoringPosition();
                    linearSlideSubsystem.lowScoringPosition();
                }
                else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
                    scoringState = ScoringState.SCORING_POSITION;
                    elbowSubsystem.mediumScoringPosition();
                    linearSlideSubsystem.mediumScoringPosition();
                }
                else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    scoringState = ScoringState.SCORING_POSITION;
                    elbowSubsystem.highScoringPosition();
                    linearSlideSubsystem.highScoringPosition();
                }
                break;
            case SCORING_POSITION:
                runPIDControllers();
                if (elbowSubsystem.isAtTarget()) scoringState = ScoringState.LOADED_DRIVING;
                break;
            case DRIVING:
                intakeSubsystem.stop();
                intakeSubsystem.mediumPosition();
                linearSlideSubsystem.retract();
                elbowSubsystem.drivingPosition();
                runPIDControllers();
                break;
            case MANUAL:
                runManually();
                break;

        }
    }

    public void runManually() {

        driveLoop();

        // Elbow Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.X)) elbowSubsystem.raiseManually();
        else if (operatorGamepad.getButton(GamepadKeys.Button.Y)) elbowSubsystem.lowerManually();
        else elbowSubsystem.stop();
        elbowSubsystem.printPosition(opMode.telemetry);
        opMode.telemetry.update();

        // Intake Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) intakeSubsystem.mediumPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) intakeSubsystem.downPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) intakeSubsystem.intake();
        else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) intakeSubsystem.outtake();
        else intakeSubsystem.stop();

        // Linear Slide Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) linearSlideSubsystem.extendManually();
        else if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) linearSlideSubsystem.retractManually();
        else linearSlideSubsystem.stop();

        // Claw Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.A)) clawSubsystem.openClaw();
        if (operatorGamepad.getButton(GamepadKeys.Button.B)) clawSubsystem.closeClaw();

        // Drone subsystem
//        if (driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) droneSubsystem.release();
//        if (driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) droneSubsystem.startPosition();

        // Winch subsystem
        if (driverGamepad.getButton(GamepadKeys.Button.A)) winchSubsystem.winch();
        else if (driverGamepad.getButton(GamepadKeys.Button.B)) winchSubsystem.unwinch();
        else winchSubsystem.stop();
    }

    public void runPIDControllers() {
        if (!usingPIDControllers) return;
        elbowSubsystem.runPID();
        linearSlideSubsystem.runPID();
    }

    public void intakePosition() {
        elbowSubsystem.intakePosition();
        linearSlideSubsystem.retract();
        clawSubsystem.openClaw();
    }

    private void alignLeft() {

    }

    private void alignCenter() {

    }

    private void alignRight() {

    }

    private void stepLeft() {
        driveSubsystem.halfStepLeft();
        driveState = DriveState.STEPPING_LEFT;
    }

    private void stepRight() {
        driveSubsystem.halfStepRight();
        driveState = DriveState.STEPPING_RIGHT;
    }

    private boolean isPressed(float controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }

    private void wait(int ms) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() <= ms) {}
    }

    public ElbowSubsystem getElbowSubsystem() {
        return elbowSubsystem;
    }
}
