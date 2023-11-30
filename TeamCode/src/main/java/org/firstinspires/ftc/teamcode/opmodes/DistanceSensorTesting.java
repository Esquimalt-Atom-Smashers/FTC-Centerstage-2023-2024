package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
