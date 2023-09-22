package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name="Main Opmode", group = "Linear Opmode")
public class MainOpMode extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this);

        // Perform actions that happen before the Robot starts

        robot.start();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }
}
