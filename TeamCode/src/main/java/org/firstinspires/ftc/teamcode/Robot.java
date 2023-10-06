package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Instructions.InstructionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class Robot {
    // OpMode
    private final OpMode opMode;

    // Gamepads
    private final Gamepad driverGamepad;
    private final Gamepad operatorGamepad;

    //Define subsystems here.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElbowSubsystem elbowSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final LinearSlideSubsystem linearSlideSubsystem;

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
        SCORING,
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
        driverGamepad = opMode.gamepad1;
        operatorGamepad = opMode.gamepad2;

        // Initialize the subsystems
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap);
        clawSubsystem = new ClawSubsystem(opMode.hardwareMap);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap);

        instructionExecutor = new InstructionExecutor();
    }

    public Robot(OpMode opMode) {
        this(opMode, true);
    }

    // Perform actions that happen when the robot starts
    public void start() {
        linearSlideSubsystem.retract();
    }

    // Main robot control
    public void run() {
        // Control all of the subsystems

        // Drive subsystem controls (driver):
        // Left joystick up and down moves the robot forward and back
        // Left joystick left and right moves the robot left and right
        // Right joystick left and right turns the robot left and right
        driveLoop();

        // FOR DRIVERS:
        // Press A to enter intake mode
        // While in intake mode press X to pick up
        // Press B to enter driving mode


        if (operatorGamepad.a) scoringState = ScoringState.INTAKE;
        if (operatorGamepad.b) {
            scoringState = ScoringState.LOADED_DRIVING;
            intakeSubsystem.stop();
            intakeSubsystem.mediumPosition();
            elbowSubsystem.drivingPosition();
        }
//        if (operatorGamepad.b) scoringState = ScoringState.DRIVING;

        if (operatorGamepad.left_stick_y <= -0.1) elbowSubsystem.lowerManually();
        else if (operatorGamepad.left_stick_y >= 0.1) elbowSubsystem.raiseManually();
        else elbowSubsystem.stop();

        if (operatorGamepad.right_stick_y >= 0.1) linearSlideSubsystem.retractManually();
        else if (operatorGamepad.right_stick_y <= -0.1) linearSlideSubsystem.extendManually();
        else linearSlideSubsystem.stop();

        if (operatorGamepad.y) clawSubsystem.openClaw();

        if (operatorGamepad.dpad_up) {
            elbowSubsystem.testPosition();
            linearSlideSubsystem.testPosition();
        }

        scoringLoop();

        opMode.telemetry.addData("Driving State", driveState);
        opMode.telemetry.addData("Scoring State", scoringState);
        elbowSubsystem.printPosition(opMode.telemetry);
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
                driveSubsystem.drive(driverGamepad.left_stick_y, -driverGamepad.left_stick_x, -driverGamepad.right_stick_x);
                if (driverGamepad.a) {
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
                if (operatorGamepad.x) {
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
            case LOADED_DRIVING:
                runPIDControllers();
                break;
            case SCORING:
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
        if (operatorGamepad.x) elbowSubsystem.raiseManually();
        else if (operatorGamepad.y) elbowSubsystem.lowerManually();
        else elbowSubsystem.stop();

        // Intake Subsystem
        if (operatorGamepad.dpad_up) intakeSubsystem.mediumPosition();
        if (operatorGamepad.dpad_down) intakeSubsystem.downPosition();
        if (operatorGamepad.dpad_left) intakeSubsystem.intake();
        else if (operatorGamepad.dpad_right) intakeSubsystem.outtake();
        else intakeSubsystem.stop();

        // Linear Slide Subsystem
        if (operatorGamepad.right_bumper) linearSlideSubsystem.extendManually();
        else if (operatorGamepad.left_bumper) linearSlideSubsystem.retractManually();
        else linearSlideSubsystem.stop();

        // Claw Subsystem
        if (operatorGamepad.a) clawSubsystem.openClaw();
        if (operatorGamepad.b) clawSubsystem.closeClaw();

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
}
