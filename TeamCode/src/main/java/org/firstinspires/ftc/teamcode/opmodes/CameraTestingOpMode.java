package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Camera Testing Opmode", group="Testing")
public class CameraTestingOpMode extends LinearOpMode {

    ExposureControl exposureControl;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);

//        exposureControl = cameraSubsystem.getVisionPortal().getCameraControl(ExposureControl.class);
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        exposureControl.setExposure(30, TimeUnit.MILLISECONDS);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            cameraSubsystem.detectAndPrintAprilTags(telemetry);
            telemetry.update();
        }
    }
}
