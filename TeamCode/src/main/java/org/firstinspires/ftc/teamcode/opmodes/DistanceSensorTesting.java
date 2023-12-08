package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(group = "testing")
public class DistanceSensorTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput sensor = hardwareMap.get(AnalogInput.class, "distanceSensor");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }
}
