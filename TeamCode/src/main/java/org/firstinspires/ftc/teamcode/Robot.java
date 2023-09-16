package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Instructions.InstructionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class Robot {
    private final OpMode opMode;

    private final Gamepad driverGamepad;
    private final Gamepad operatorGamepad;

    //Define subsystems here.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final LinearSlideSubsystem linearSlideSubsystem;

    private final InstructionExecutor instructionExecutor;


    //Define gamepads here.

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        driverGamepad = opMode.gamepad1;
        operatorGamepad = opMode.gamepad2;

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap, opMode.telemetry);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap, opMode.telemetry);
        clawSubsystem = new ClawSubsystem(opMode.hardwareMap, opMode.telemetry);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap, opMode.telemetry);

        instructionExecutor = new InstructionExecutor();
    }

    public void run() {
        // Control all of the subsystems

        // Drive subsystem controls (driver):
        // Left joystick up and down moves the robot forward and back
        // Left joystick left and right moves the robot left and right
        // Right joystick left and right turns the robot left and right
        driveSubsystem.drive(driverGamepad.left_stick_y, driverGamepad.left_stick_x, driverGamepad.right_stick_x);

        // Instruction controls (driver):
        // Left trigger adds a left align
        // Right trigger adds a right align
        // Both triggers add a center align
        // Left bumper adds a step left
        // Right bumper adds a step right
        // A enters the instructions
        // B cancels the instructions
        if (isPressed(driverGamepad.left_trigger) && isPressed(driverGamepad.right_trigger))
            instructionExecutor.addInstruction(this::alignCenter);
        else if (isPressed(driverGamepad.left_trigger))
            instructionExecutor.addInstruction(this::alignLeft);
        else if (isPressed(driverGamepad.right_trigger))
            instructionExecutor.addInstruction(this::alignRight);
        if (driverGamepad.left_bumper) instructionExecutor.addInstruction(this::stepLeft);
        if (driverGamepad.right_bumper) instructionExecutor.addInstruction(this::stepRight);
        if (driverGamepad.a) instructionExecutor.executeInstructions();
        if (driverGamepad.b) instructionExecutor.clearInstructions();

        // Intake subsystem controls (operator):
        // A intakes the robot
        // B outtakes the robot
        // X lifts the intake
        // Y lowers the intake
        if (operatorGamepad.a) {
            intakeSubsystem.intake();
        } else if (operatorGamepad.b) {
            intakeSubsystem.outtake();
        } else intakeSubsystem.stop();
        if (operatorGamepad.x) intakeSubsystem.raiseIntake();
        if (operatorGamepad.y) intakeSubsystem.lowerIntake();

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


    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    private void alignLeft() {

    }

    private void alignCenter() {

    }

    private void alignRight() {

    }

    private void stepLeft() {

    }

    private void stepRight() {

    }

    private boolean isPressed(float controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}
