package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

@TeleOp(name="Camera Testing Opmode", group="Linear Opmode")
public class CameraTestingOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            cameraSubsystem.detectAndPrint();
            telemetry.update();
        }
    }
}
