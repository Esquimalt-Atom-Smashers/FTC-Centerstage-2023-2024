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
    private final OpMode opMode;

    private final Gamepad driverGamepad;
    private final Gamepad operatorGamepad;

    //Define subsystems here.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElbowSubsystem elbowSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final LinearSlideSubsystem linearSlideSubsystem;

    private final InstructionExecutor instructionExecutor;

    enum DriveState {
        DRIVER_CONTROLLED,
        SNAPPING,
        DETECTING_TAG,
        CENTERING_TAG,
        STEPPING_LEFT,
        STEPPING_RIGHT
    }

    private DriveState driveState = DriveState.DRIVER_CONTROLLED;

    //Define gamepads here.

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        driverGamepad = opMode.gamepad1;
        operatorGamepad = opMode.gamepad2;

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap, opMode.telemetry);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap, opMode.telemetry);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap, opMode.telemetry);
        clawSubsystem = new ClawSubsystem(opMode.hardwareMap, opMode.telemetry);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap, opMode.telemetry);

        instructionExecutor = new InstructionExecutor();
    }

    public void start() {
        linearSlideSubsystem.retract();
    }

    public void run() {
        // Control all of the subsystems

        // Drive subsystem controls (driver):
        // Left joystick up and down moves the robot forward and back
        // Left joystick left and right moves the robot left and right
        // Right joystick left and right turns the robot left and right
        driveLoop();

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

        // Intake subsystem controls (operator):
        // A intakes the robot
        // B outtakes the robot
        // X lifts the intake
        // Y lowers the intake
//        if (operatorGamepad.a) {
//            intakeSubsystem.intake();
//        } else if (operatorGamepad.b) {
//            intakeSubsystem.outtake();
//        } else intakeSubsystem.stop();
//        if (operatorGamepad.x) intakeSubsystem.raiseIntake();
//        if (operatorGamepad.y) intakeSubsystem.lowerIntake();
//        if (operatorGamepad.x) intakeSubsystem.printPosition();


        // Claw subsystem controls (operator):
        // A opens the claw
        // B closes the claw
        if (operatorGamepad.a) clawSubsystem.openClaw();
        if (operatorGamepad.b) clawSubsystem.closeClaw();

        // Elbow subsystem controls (operator):
        // D-pad down sets it to default
        // D-pad left sets it to low
        // D-pad up sets it to medium
        // D-pad right sets it to high

        if (operatorGamepad.dpad_up) intakeSubsystem.raiseIntake();
        if (operatorGamepad.dpad_down) intakeSubsystem.lowerIntake();
        if (operatorGamepad.dpad_left) intakeSubsystem.intake();
        else if (operatorGamepad.dpad_right) intakeSubsystem.outtake();
        else intakeSubsystem.stop();

//        driveSubsystem.printPower();
//        opMode.telemetry.update();

        if (operatorGamepad.right_bumper) {
            linearSlideSubsystem.extend();
        } else if (operatorGamepad.left_bumper) {
            linearSlideSubsystem.retract();
        } else linearSlideSubsystem.stop();

        linearSlideSubsystem.run();

        if (operatorGamepad.x) elbowSubsystem.spin();
        else if (operatorGamepad.y) elbowSubsystem.counterSpin();
        else elbowSubsystem.stop();
        elbowSubsystem.run();

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

//    public DriveSubsystem getDriveSubsystem() {
//        return driveSubsystem;
//    }
}
