package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class Robot {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ElbowSubsystem elbow;
    private final LinearSlideSubsystem slide;
    private final WristSubsystem wrist;
    private final ClawSubsystem claw;

    private final Gamepad driverGamepad;
    private final Gamepad operatorGamepad;

    public Robot(LinearOpMode opMode) {
        drive = new DriveSubsystem(opMode.hardwareMap, opMode.telemetry);
        intake = new IntakeSubsystem(opMode.hardwareMap, opMode.telemetry);
        elbow = new ElbowSubsystem(opMode.hardwareMap, opMode.telemetry);
        slide = new LinearSlideSubsystem(opMode.hardwareMap, opMode.telemetry);
        wrist = new WristSubsystem(opMode.hardwareMap, opMode.telemetry);
        claw = new ClawSubsystem(opMode.hardwareMap, opMode.telemetry);

        driverGamepad = opMode.gamepad1;
        operatorGamepad = opMode.gamepad2;
    }

    public void run() {
        drive.drive(-driverGamepad.left_stick_y, driverGamepad.left_stick_x, driverGamepad.right_stick_x);

        if (driverGamepad.a) intake.intake();
        if (driverGamepad.b) intake.outtake();

        if (operatorGamepad.dpad_up) elbow.rotateNext();
        if (operatorGamepad.dpad_down) elbow.rotatePrev();

        if (driverGamepad.dpad_up) slide.moveNext();
        if (driverGamepad.dpad_down) slide.movePrev();

        if (operatorGamepad.a) wrist.dropoff();
        if (operatorGamepad.b) wrist.pickup();

        if (operatorGamepad.x) claw.close();
        if (operatorGamepad.y) claw.open();
    }

    public void pickupAndAlign() {
        setSubsystemDefaultPosition();
    }

    public void placePixelLow() {
        setSubsystemDefaultPosition();

        claw.close();
        wait(250);
        wrist.dropoff();
        wait(250);
        elbow.rotateTo(ElbowSubsystem.TargetRotation.LOW);
        wait(500);
        slide.setPosition(LinearSlideSubsystem.TargetPosition.QUARTER);
        wait(500);
    }

    public void placePixelMedium() {
        setSubsystemDefaultPosition();

        claw.close();
        wait(250);
        wrist.dropoff();
        wait(250);
        elbow.rotateTo(ElbowSubsystem.TargetRotation.MEDIUM);
        wait(500);
        slide.setPosition(LinearSlideSubsystem.TargetPosition.HALF);
        wait(500);
    }

    /**
     * Sets the default position for all the subsystems. i.e Claw is set to open
     */
    public void setSubsystemDefaultPosition() {
        claw.open();
        wait(250);
        slide.setPosition(LinearSlideSubsystem.TargetPosition.DEFAULT);
        wait(500);
        wrist.dropoff();
        wait(250);
        elbow.rotateTo(ElbowSubsystem.TargetRotation.DEFAULT);
        wait(500);
    }

    private void wait(int ms) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() <= ms) { }
    }
}
